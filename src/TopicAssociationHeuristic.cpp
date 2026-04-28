#include "gz_ros2_bridge_manager/TopicAssociationHeuristic.hh"

#include <algorithm>
#include <array>
#include <cctype>

namespace gz_ros2_bridge_manager
{

const char *categoryName(AssociationCategory c)
{
  switch (c)
  {
    case AssociationCategory::ExactModelPath:           return "ExactModelPath";
    case AssociationCategory::ContainsSanitizedModelName: return "ContainsSanitizedModelName";
    case AssociationCategory::ContainsModelName:        return "ContainsModelName";
    case AssociationCategory::CompatibleButUnassigned:  return "CompatibleButUnassigned";
    case AssociationCategory::Unsupported:              return "Unsupported";
  }
  return "Unknown";
}

// ---- Sanitisation -----------------------------------------------------------

std::string TopicAssociationHeuristic::sanitizeName(const std::string &name)
{
  std::string s;
  s.reserve(name.size());
  for (char c : name)
  {
    if (std::isalnum(static_cast<unsigned char>(c)))
      s.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    else
      s.push_back('_');
  }

  // Collapse repeated underscores.
  std::string collapsed;
  collapsed.reserve(s.size());
  bool prevUnderscore = false;
  for (char c : s)
  {
    if (c == '_')
    {
      if (!prevUnderscore)
        collapsed.push_back('_');
      prevUnderscore = true;
    }
    else
    {
      collapsed.push_back(c);
      prevUnderscore = false;
    }
  }

  // Trim leading/trailing underscores.
  const auto first = collapsed.find_first_not_of('_');
  if (first == std::string::npos)
    return {};
  const auto last = collapsed.find_last_not_of('_');
  return collapsed.substr(first, last - first + 1);
}

// ---- Generic-topic detection -----------------------------------------------

namespace
{
constexpr std::array<const char *, 9> kGenericExact = {
  "/clock", "/tf", "/tf_static", "/joint_states",
  "/scan",  "/imu", "/points",   "/odom",  "/cmd_vel"
};

constexpr std::array<const char *, 7> kGenericPrefix = {
  "/imu/", "/scan/", "/camera/", "/depth/", "/image/", "/points/", "/odom/"
};
}  // namespace

bool TopicAssociationHeuristic::isGenericTopic(const std::string &topic)
{
  for (const auto *t : kGenericExact)
    if (topic == t)
      return true;
  for (const auto *p : kGenericPrefix)
  {
    const std::string prefix = p;
    if (topic.size() >= prefix.size() &&
        topic.compare(0, prefix.size(), prefix) == 0)
      return true;
  }
  return false;
}

// ---- Token-bounded substring search ----------------------------------------

bool TopicAssociationHeuristic::tokenContains(const std::string &haystack,
                                              const std::string &needle)
{
  if (needle.empty() || haystack.empty() || needle.size() > haystack.size())
    return false;

  auto isIdent = [](char c)
  {
    return std::isalnum(static_cast<unsigned char>(c)) || c == '_';
  };

  size_t pos = 0;
  while ((pos = haystack.find(needle, pos)) != std::string::npos)
  {
    const bool leftOk =
        (pos == 0) || !isIdent(haystack[pos - 1]);
    const bool rightOk =
        (pos + needle.size() == haystack.size()) ||
        !isIdent(haystack[pos + needle.size()]);
    if (leftOk && rightOk)
      return true;
    ++pos;
  }
  return false;
}

// ---- Exact /model/ path match ----------------------------------------------

bool TopicAssociationHeuristic::hasExactModelPath(const std::string &topic,
                                                  const std::string &worldName,
                                                  const std::string &modelName)
{
  if (modelName.empty())
    return false;

  // /model/<name>/   (also matches /world/<w>/model/<name>/ by substring)
  const std::string scoped = "/model/" + modelName + "/";
  if (topic.find(scoped) != std::string::npos)
    return true;

  // Trailing form: ".../model/<name>" with no trailing slash.
  const std::string scopedTail = "/model/" + modelName;
  if (topic.size() >= scopedTail.size() &&
      topic.compare(topic.size() - scopedTail.size(),
                    scopedTail.size(), scopedTail) == 0)
    return true;

  // /world/<w>/model/<name>/  — already covered by the substring above,
  // but check explicitly when worldName is provided to keep the logic clear.
  if (!worldName.empty())
  {
    const std::string worldScoped =
        "/world/" + worldName + "/model/" + modelName + "/";
    if (topic.find(worldScoped) != std::string::npos)
      return true;
  }
  return false;
}

// ---- Main classification ----------------------------------------------------

namespace
{

struct ClassificationOut
{
  AssociationCategory category = AssociationCategory::CompatibleButUnassigned;
  std::string         label    = "Unassigned";
};

// Classifies a single topic against a single model. Does not handle ambiguity.
ClassificationOut classifyForModel(const GzTopicEntry &t,
                                   const std::string &model,
                                   const std::string &sanitizedModel,
                                   const std::string &worldName)
{
  ClassificationOut out;
  if (model.empty())
    return out;  // No selection → unassigned

  if (TopicAssociationHeuristic::hasExactModelPath(t.topicName, worldName, model))
  {
    out.category = AssociationCategory::ExactModelPath;
    out.label    = "Strong (model path)";
    return out;
  }

  if (!sanitizedModel.empty() && sanitizedModel != model &&
      TopicAssociationHeuristic::tokenContains(t.topicName, sanitizedModel))
  {
    out.category = AssociationCategory::ContainsSanitizedModelName;
    out.label    = "Medium (sanitized name)";
    return out;
  }

  if (TopicAssociationHeuristic::tokenContains(t.topicName, model))
  {
    out.category = AssociationCategory::ContainsModelName;
    out.label    = "Medium (model name)";
    return out;
  }

  return out;
}

// True if any other model name is also a token-match in this topic AND
// is more specific (longer) than the selected model. Used to flag ambiguity.
bool isAmbiguousAgainstOtherModels(const GzTopicEntry &t,
                                   const std::string &selectedModel,
                                   const std::vector<std::string> &allModels)
{
  for (const auto &other : allModels)
  {
    if (other == selectedModel || other.empty())
      continue;
    // Conflict criterion: another model name appears as a token in the topic
    // AND it is at least as long as the selected name (more or equally
    // specific). Shorter names can be substrings of longer ones, but our
    // tokenContains already prevents matching across identifier boundaries.
    if (other.size() >= selectedModel.size() &&
        TopicAssociationHeuristic::tokenContains(t.topicName, other))
      return true;
  }
  return false;
}

}  // namespace

AssociationResult TopicAssociationHeuristic::associate(
    const std::string &selectedModel,
    const std::string &worldName,
    const std::vector<GzTopicEntry> &topics,
    const std::vector<std::string> &allModels) const
{
  AssociationResult result;
  const std::string sanitizedModel =
      selectedModel.empty() ? std::string{} : sanitizeName(selectedModel);

  bool sawAmbiguous = false;

  for (const auto &t : topics)
  {
    BridgeTopicCandidate c;
    c.gzTopic    = t.topicName;
    c.gzType     = t.gzMsgType;
    c.ros2Type   = t.ros2MsgType;
    c.bridgeSpec = t.bridgeSpec;
    c.bridgeable = t.bridgeable;
    c.isGeneric  = isGenericTopic(t.topicName);

    if (!t.bridgeable)
    {
      c.category = AssociationCategory::Unsupported;
      c.confidenceLabel = c.gzType.empty()
                            ? "Unknown gz type"
                            : "Unsupported gz type";
      c.checked = false;
      result.unsupported.push_back(std::move(c));
      continue;
    }

    // Generic topics are never auto-associated to a model unless the path
    // explicitly contains the model name. We still run the classifier to
    // detect path-level matches like /model/<name>/scan.
    const ClassificationOut cls = classifyForModel(
        t, selectedModel, sanitizedModel, worldName);

    // For generic topics with no path-level match, force-unassign.
    if (c.isGeneric && cls.category != AssociationCategory::ExactModelPath)
    {
      c.category = AssociationCategory::CompatibleButUnassigned;
      c.confidenceLabel = "Generic / global";
      c.checked = false;
      if (t.topicName == "/clock")
        c.warning = "Global clock — usually bridged separately.";
      result.unassigned.push_back(std::move(c));
      continue;
    }

    c.category        = cls.category;
    c.confidenceLabel = cls.label;

    const bool isAssociated =
        cls.category != AssociationCategory::CompatibleButUnassigned;

    if (isAssociated)
    {
      // Check ambiguity only for non-path matches; ExactModelPath wins
      // unambiguously by construction.
      if (cls.category != AssociationCategory::ExactModelPath &&
          isAmbiguousAgainstOtherModels(t, selectedModel, allModels))
      {
        c.ambiguous = true;
        c.warning   = "Ambiguous: another model name also matches this topic.";
        c.checked   = false;
        sawAmbiguous = true;
        result.unassigned.push_back(std::move(c));
        continue;
      }

      c.checked = true;
      result.associated.push_back(std::move(c));
    }
    else
    {
      c.confidenceLabel = "Unassigned";
      c.checked = false;
      result.unassigned.push_back(std::move(c));
    }
  }

  if (sawAmbiguous)
    result.warnings.push_back(
        "Some topics matched multiple model names and were not auto-checked.");

  return result;
}

}  // namespace gz_ros2_bridge_manager

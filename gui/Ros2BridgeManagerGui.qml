import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

// Context property: bridgeManager → Ros2BridgeManagerGui (C++)
Rectangle {
  id: root
  color: "transparent"
  anchors.fill: parent
  implicitWidth: 400
  implicitHeight: 700
  Layout.minimumWidth: 320
  Layout.minimumHeight: 320
  Layout.preferredHeight: 700

  // ---- JS helpers --------------------------------------------------------

  function topicCountLabel() {
    var assoc  = bridgeManager.associatedTopics.length
    var unassn = bridgeManager.unassignedTopics.length
    var unsup  = bridgeManager.unsupportedTopics.length
    return "associated: " + assoc +
           "  •  additional: " + unassn +
           "  •  unsupported: " + unsup
  }

  // Human-readable match source labels and colours for the sensor detail view.
  function matchSourceLabel(src) {
    if (src === "EcmSensorTopicExact")  return "SensorTopic exact";
    if (src === "EcmSensorTopicPrefix") return "SensorTopic prefix";
    if (src === "EcmStandardPrefix")    return "Gazebo standard prefix";
    if (src === "Unresolved")           return "Unresolved";
    return src;
  }

  function matchSourceColor(src) {
    if (src === "EcmSensorTopicExact")  return "#0d47a1";
    if (src === "EcmSensorTopicPrefix") return "#1565c0";
    if (src === "EcmStandardPrefix")    return "#1976d2";
    if (src === "Unresolved")           return "#bf360c";
    return "#757575";
  }

  // ---- Reusable flat topic row (used in heuristic / additional sections) ----
  component TopicRow: Rectangle {
    id: rowRoot
    property var entry
    property bool checkable: true

    width: ListView.view ? ListView.view.width : implicitWidth
    height: 30
    color: index % 2 === 0 ? "#ffffff" : "#f5f5f5"

    RowLayout {
      anchors { fill: parent; leftMargin: 4; rightMargin: 4 }
      spacing: 4

      CheckBox {
        Layout.alignment: Qt.AlignVCenter
        Layout.preferredWidth: 22
        padding: 0
        checked: entry.checked
        enabled: rowRoot.checkable && entry.bridgeable
        onToggled: bridgeManager.setTopicChecked(entry.topic, checked)
      }

      Item {
        Layout.preferredWidth: 14
        Layout.alignment: Qt.AlignVCenter
        height: 14
        Label {
          anchors.centerIn: parent
          font.pixelSize: 10
          text: entry.ambiguous ? "?" : (entry.isGeneric ? "*" : "")
          color: entry.ambiguous ? "#c62828" : "#1565c0"
          font.bold: true
        }
      }

      Label {
        text: entry.topic
        font.pixelSize: 10; font.family: "monospace"
        elide: Text.ElideMiddle
        Layout.fillWidth: true
        Layout.preferredWidth: 140
        ToolTip.visible: topicHover.containsMouse
        ToolTip.text: entry.warning.length > 0
                      ? entry.topic + "\n" + entry.warning
                      : entry.topic
        ToolTip.delay: 400

        MouseArea {
          id: topicHover
          anchors.fill: parent
          hoverEnabled: true
          acceptedButtons: Qt.NoButton
        }
      }

      Label {
        text: {
          var t = entry.gzType
          return t.length > 0 ? t.replace("gz.msgs.", "") : "?"
        }
        font.pixelSize: 10; font.family: "monospace"
        color: "#1565c0"
        elide: Text.ElideRight
        Layout.preferredWidth: 90
      }

      Label {
        text: entry.confidence
        font.pixelSize: 9; font.italic: true
        color: {
          if (entry.category === "EcmConfirmed")                return "#0d47a1"
          if (entry.category === "ExactModelPath")              return "#1b5e20"
          if (entry.category === "ContainsSanitizedModelName")  return "#2e7d32"
          if (entry.category === "ContainsModelName")           return "#33691e"
          if (entry.ambiguous)                                  return "#c62828"
          if (entry.isGeneric)                                  return "#ef6c00"
          return "#757575"
        }
        elide: Text.ElideRight
        Layout.preferredWidth: 100
      }
    }
  }

  // ================================================================
  ScrollView {
    id: mainScroll
    anchors.fill: parent
    contentWidth: availableWidth
    clip: true
    ScrollBar.vertical.policy: ScrollBar.AsNeeded

    ColumnLayout {
      id: mainCol
      width: mainScroll.availableWidth
      spacing: 6

      Item { implicitHeight: 4 }

      // ── 1. Header ─────────────────────────────────────────────────
      RowLayout {
        Layout.leftMargin: 10; Layout.rightMargin: 10
        Layout.fillWidth: true

        Label {
          text: "ROS 2 Bridge Manager"
          font.bold: true; font.pixelSize: 14
          Layout.fillWidth: true
        }

        CheckBox {
          text: "Auto"
          font.pixelSize: 10; padding: 4
          checked: bridgeManager.autoRefresh
          onToggled: bridgeManager.setAutoRefresh(checked)
          ToolTip.visible: hovered
          ToolTip.text: "Auto-refresh every ~2.5 s"
          ToolTip.delay: 400
        }

        Button {
          text: bridgeManager.busy ? "…" : "Refresh"
          enabled: !bridgeManager.busy
          implicitWidth: 70; font.pixelSize: 11
          onClicked: bridgeManager.refresh()
        }
      }

      // ── 2. Status bar ─────────────────────────────────────────────
      Rectangle {
        Layout.leftMargin: 10; Layout.rightMargin: 10
        Layout.fillWidth: true
        implicitHeight: statusCol.implicitHeight + 10
        color: bridgeManager.worldName.length > 0 ? "#e8f5e9" : "#fce4ec"
        radius: 4

        ColumnLayout {
          id: statusCol
          anchors {
            left: parent.left; right: parent.right
            verticalCenter: parent.verticalCenter
            leftMargin: 8; rightMargin: 8
          }
          spacing: 2

          RowLayout {
            spacing: 8

            BusyIndicator {
              running: bridgeManager.busy
              visible: bridgeManager.busy
              width: 16; height: 16
            }

            Label {
              text: bridgeManager.statusText
              font.pixelSize: 11
              color: bridgeManager.worldName.length > 0 ? "#1b5e20" : "#b71c1c"
              wrapMode: Text.Wrap; Layout.fillWidth: true
            }

            Label {
              visible: bridgeManager.lastRefreshTime.length > 0
              text: "↻ " + bridgeManager.lastRefreshTime
              font.pixelSize: 10; color: "#558b2f"
            }
          }
        }
      }

      // ── 3. Model-gone warning ─────────────────────────────────────
      Rectangle {
        Layout.leftMargin: 10; Layout.rightMargin: 10
        Layout.fillWidth: true
        implicitHeight: modelGoneLabel.implicitHeight + 12
        color: "#fff3e0"
        border.color: "#ef6c00"; border.width: 1
        radius: 4
        visible: bridgeManager.modelGoneWarning.length > 0

        Label {
          id: modelGoneLabel
          anchors {
            top: parent.top; left: parent.left; right: parent.right
            topMargin: 6; leftMargin: 8; rightMargin: 8
          }
          text: "⚠ " + bridgeManager.modelGoneWarning
          font.pixelSize: 10; color: "#bf360c"
          wrapMode: Text.Wrap
        }
      }

      // ── 4. Model selector + ECM status ────────────────────────────
      Rectangle {
        visible: bridgeManager.modelNames.length > 0 ||
                 bridgeManager.selectedModel.length > 0
        Layout.leftMargin: 10; Layout.rightMargin: 10
        Layout.fillWidth: true
        implicitHeight: modelCol.implicitHeight + 16
        color: "#fafafa"
        border.color: "#e0e0e0"; border.width: 1
        radius: 4

        ColumnLayout {
          id: modelCol
          anchors {
            top: parent.top; left: parent.left; right: parent.right
            topMargin: 8; leftMargin: 8; rightMargin: 8
          }
          spacing: 4

          RowLayout {
            spacing: 6; Layout.fillWidth: true

            Label { text: "Model:"; font.bold: true; font.pixelSize: 12; color: "#555" }

            ComboBox {
              id: modelCombo
              Layout.fillWidth: true
              font.pixelSize: 11
              model: ["(no model — manual selection)"].concat(bridgeManager.modelNames)

              currentIndex: {
                var idx = bridgeManager.modelNames.indexOf(bridgeManager.selectedModel)
                return idx >= 0 ? idx + 1 : 0
              }

              onActivated: {
                var name = (currentIndex === 0) ? "" : bridgeManager.modelNames[currentIndex - 1]
                bridgeManager.selectModel(name)
              }

              Connections {
                target: bridgeManager
                function onSelectedModelChanged() {
                  var idx = bridgeManager.modelNames.indexOf(bridgeManager.selectedModel)
                  modelCombo.currentIndex = (idx >= 0) ? idx + 1 : 0
                }
              }
            }

            Button {
              text: "Reset"; font.pixelSize: 10
              implicitWidth: 56; implicitHeight: 24
              ToolTip.visible: hovered
              ToolTip.text: "Reset this model's manual selections to heuristic defaults"
              ToolTip.delay: 400
              onClicked: bridgeManager.resetCurrentModelSelection()
            }
          }

          // ECM discovery status (compact inline indicator)
          RowLayout {
            Layout.fillWidth: true
            spacing: 6

            Rectangle {
              width: 8; height: 8; radius: 4
              color: bridgeManager.ecmAvailable ? "#43a047" : "#bdbdbd"
            }

            Label {
              text: bridgeManager.ecmAvailable
                    ? ("Sensor discovery: ECM active  •  " +
                       bridgeManager.sensorDiscoveryStatus)
                    : "Sensor discovery: ECM unavailable — using topic-name heuristic"
              font.pixelSize: 10
              color: bridgeManager.ecmAvailable ? "#2e7d32" : "#757575"
              wrapMode: Text.Wrap; Layout.fillWidth: true
            }
          }

          Label {
            visible: bridgeManager.warnings.length > 0
            text: "⚠ " + bridgeManager.warnings.join("\n⚠ ")
            font.pixelSize: 10; color: "#b71c1c"
            wrapMode: Text.Wrap; Layout.fillWidth: true
          }

          Label {
            text: topicCountLabel()
            font.pixelSize: 10; color: "#616161"
          }

          Label {
            text: "Selections are remembered per model during this session."
            font.pixelSize: 9; font.italic: true; color: "#9e9e9e"
          }
        }
      }

      // ── 5. Detected sensors in selected model (ECM sensorTree) ────
      Rectangle {
        id: ecmCard
        Layout.leftMargin: 10; Layout.rightMargin: 10
        Layout.fillWidth: true
        implicitHeight: ecmCol.implicitHeight + 16
        color: "#e8f5e9"
        border.color: "#43a047"; border.width: 1
        radius: 4
        visible: bridgeManager.ecmAvailable && bridgeManager.sensorTree.length > 0

        property bool expanded: true

        ColumnLayout {
          id: ecmCol
          anchors {
            top: parent.top; left: parent.left; right: parent.right
            topMargin: 8; leftMargin: 8; rightMargin: 8
          }
          spacing: 4

          // Section header with collapse toggle
          Item {
            Layout.fillWidth: true
            implicitHeight: ecmHeader.implicitHeight + 4

            Label {
              id: ecmHeader
              text: (ecmCard.expanded ? "▼" : "▶") +
                    "  Detected sensors in selected model  (" +
                    bridgeManager.sensorTree.length + ")"
              font.bold: true; font.pixelSize: 12; color: "#1b5e20"
            }
            MouseArea {
              anchors.fill: parent
              cursorShape: Qt.PointingHandCursor
              onClicked: ecmCard.expanded = !ecmCard.expanded
            }
          }

          // Per-sensor rows
          Repeater {
            model: ecmCard.expanded ? bridgeManager.sensorTree : []

            delegate: Rectangle {
              id: sensorCard
              Layout.fillWidth: true
              implicitHeight: sensorCol.implicitHeight + 10
              color: modelData.resolved ? "#f1f8e9" : "#fff8e1"
              radius: 3
              border.color: modelData.resolved ? "#a5d6a7" : "#ffe082"
              border.width: 1

              // Pin outer modelData to a named property so inner Repeaters can reach it.
              property var sensorData: modelData

              ColumnLayout {
                id: sensorCol
                anchors {
                  left: parent.left; right: parent.right; top: parent.top
                  leftMargin: 6; rightMargin: 6; topMargin: 4
                }
                spacing: 2

                // Identity: link / sensor  [type]  [nested?]
                RowLayout {
                  Layout.fillWidth: true
                  spacing: 4

                  Label {
                    text: sensorCard.sensorData.linkName + " / " +
                          sensorCard.sensorData.sensorName
                    font.pixelSize: 10; font.bold: true; font.family: "monospace"
                    color: "#1b5e20"
                    Layout.fillWidth: true; elide: Text.ElideRight
                  }

                  Label {
                    text: sensorCard.sensorData.sensorType
                    font.pixelSize: 9; font.italic: true; color: "#388e3c"
                  }

                  Label {
                    visible: sensorCard.sensorData.nestedModel
                    text: "nested"
                    font.pixelSize: 8; font.italic: true; color: "#1565c0"
                  }
                }

                // Match source
                Label {
                  visible: sensorCard.sensorData.resolved
                  text: "source: " + root.matchSourceLabel(sensorCard.sensorData.matchSource)
                  font.pixelSize: 9; font.italic: true
                  color: root.matchSourceColor(sensorCard.sensorData.matchSource)
                }

                // Topic prefix — declared topic takes priority over fallback
                Label {
                  visible: sensorCard.sensorData.declaredTopic.length > 0
                  text: "topic: " + sensorCard.sensorData.declaredTopic
                  font.pixelSize: 9; font.family: "monospace"; color: "#424242"
                  Layout.fillWidth: true; elide: Text.ElideMiddle
                  ToolTip.visible: dtHover.containsMouse
                  ToolTip.text: sensorCard.sensorData.declaredTopic
                  ToolTip.delay: 300
                  MouseArea { id: dtHover; anchors.fill: parent; hoverEnabled: true; acceptedButtons: Qt.NoButton }
                }
                Label {
                  visible: sensorCard.sensorData.declaredTopic.length === 0 &&
                           sensorCard.sensorData.fallbackPrefix.length > 0
                  text: "path: " + sensorCard.sensorData.fallbackPrefix
                  font.pixelSize: 9; font.family: "monospace"; color: "#616161"
                  Layout.fillWidth: true; elide: Text.ElideMiddle
                  ToolTip.visible: fpHover.containsMouse
                  ToolTip.text: sensorCard.sensorData.fallbackPrefix
                  ToolTip.delay: 300
                  MouseArea { id: fpHover; anchors.fill: parent; hoverEnabled: true; acceptedButtons: Qt.NoButton }
                }

                // Matched topic rows (checkbox + topic + gz→ros2 types)
                Repeater {
                  model: sensorCard.sensorData.matchedTopicDetails

                  delegate: RowLayout {
                    id: tdRow
                    Layout.fillWidth: true
                    Layout.leftMargin: 8
                    spacing: 3

                    // Look up live check/bridgeable state from associatedTopics.
                    // This binding re-evaluates whenever associatedTopics changes (topicsChanged).
                    property string tdTopic: modelData.topic
                    property bool tdChecked: {
                      var t = tdRow.tdTopic;
                      var list = bridgeManager.associatedTopics;
                      for (var i = 0; i < list.length; i++) {
                        if (list[i].topic === t) return list[i].checked;
                      }
                      return false;
                    }
                    property bool tdBridgeable: {
                      var t = tdRow.tdTopic;
                      var list = bridgeManager.associatedTopics;
                      for (var i = 0; i < list.length; i++) {
                        if (list[i].topic === t) return list[i].bridgeable;
                      }
                      return true;
                    }

                    CheckBox {
                      padding: 0
                      Layout.preferredWidth: 22
                      Layout.alignment: Qt.AlignVCenter
                      checked: tdRow.tdChecked
                      enabled: tdRow.tdBridgeable
                      onToggled: bridgeManager.setTopicChecked(tdRow.tdTopic, checked)
                    }

                    Label {
                      text: modelData.topic
                      font.pixelSize: 9; font.family: "monospace"; color: "#33691e"
                      Layout.fillWidth: true; elide: Text.ElideMiddle
                      ToolTip.visible: tdHover.containsMouse
                      ToolTip.text: modelData.topic
                      ToolTip.delay: 300
                      MouseArea { id: tdHover; anchors.fill: parent; hoverEnabled: true; acceptedButtons: Qt.NoButton }
                    }

                    Label {
                      text: modelData.gzType.replace("gz.msgs.", "")
                      font.pixelSize: 8; color: "#1565c0"
                      elide: Text.ElideRight; Layout.preferredWidth: 72
                    }

                    Label { text: "→"; font.pixelSize: 8; color: "#9e9e9e" }

                    Label {
                      text: {
                        var parts = modelData.ros2Type.split("/");
                        return parts.length > 0 ? parts[parts.length - 1] : modelData.ros2Type;
                      }
                      font.pixelSize: 8; color: "#5d4037"
                      elide: Text.ElideRight; Layout.preferredWidth: 80
                    }
                  }
                }

                // Unresolved warning
                Label {
                  visible: !sensorCard.sensorData.resolved &&
                            sensorCard.sensorData.warning.length > 0
                  text: "⚠ " + sensorCard.sensorData.warning
                  font.pixelSize: 9; font.italic: true; color: "#bf360c"
                  wrapMode: Text.Wrap; Layout.fillWidth: true
                }

                Item { implicitHeight: 2 }
              }
            }
          }
        }
      }

      // "No sensors for selected model" — ECM active but nothing found
      Rectangle {
        Layout.leftMargin: 10; Layout.rightMargin: 10
        Layout.fillWidth: true
        implicitHeight: noSensorLabel.implicitHeight + 12
        color: "#fff3e0"
        border.color: "#ff8f00"; border.width: 1
        radius: 4
        visible: bridgeManager.ecmAvailable &&
                 bridgeManager.selectedModel.length > 0 &&
                 bridgeManager.sensorTree.length === 0

        Label {
          id: noSensorLabel
          anchors {
            left: parent.left; right: parent.right; verticalCenter: parent.verticalCenter
            leftMargin: 8; rightMargin: 8
          }
          text: "No sensors detected for selected model."
          font.pixelSize: 10; font.italic: true; color: "#e65100"
          wrapMode: Text.Wrap
        }
      }

      // ECM unavailable banner (when model selected but ECM not active)
      Rectangle {
        Layout.leftMargin: 10; Layout.rightMargin: 10
        Layout.fillWidth: true
        implicitHeight: ecmUnavailLabel.implicitHeight + 12
        color: "#fff8e1"
        border.color: "#ffe082"; border.width: 1
        radius: 4
        visible: !bridgeManager.ecmAvailable &&
                 bridgeManager.selectedModel.length > 0

        Label {
          id: ecmUnavailLabel
          anchors {
            left: parent.left; right: parent.right; verticalCenter: parent.verticalCenter
            leftMargin: 8; rightMargin: 8
          }
          text: "⚠ ECM sensor discovery unavailable. " +
                "Falling back to topic-name heuristics."
          font.pixelSize: 10; color: "#f57f17"
          wrapMode: Text.Wrap
        }
      }

      // ── 6. Bridge command ─────────────────────────────────────────
      Rectangle {
        Layout.leftMargin: 10; Layout.rightMargin: 10
        Layout.fillWidth: true
        implicitHeight: cmdCol.implicitHeight + 16
        color: bridgeManager.bridgeCommand.length > 0 ? "#e8f5e9" : "#f5f5f5"
        border.color: bridgeManager.bridgeCommand.length > 0 ? "#66bb6a" : "#bdbdbd"
        border.width: 1
        radius: 4

        ColumnLayout {
          id: cmdCol
          anchors {
            top: parent.top; left: parent.left; right: parent.right
            topMargin: 8; leftMargin: 8; rightMargin: 8
          }
          spacing: 6

          RowLayout {
            Layout.fillWidth: true

            Label {
              text: "Bridge command  •  " + bridgeManager.selectionSummary
              font.bold: true; font.pixelSize: 12
              color: bridgeManager.bridgeCommand.length > 0 ? "#1b5e20" : "#757575"
              Layout.fillWidth: true; elide: Text.ElideRight
            }

            Button {
              text: "Copy"; font.pixelSize: 11
              implicitWidth: 56; implicitHeight: 26
              enabled: bridgeManager.bridgeCommand.length > 0
              onClicked: bridgeManager.copyBridgeCommand()
            }

            Button {
              text: "Uncheck"; font.pixelSize: 11
              implicitWidth: 70; implicitHeight: 26
              enabled: bridgeManager.checkedCurrentModelCount > 0
              ToolTip.visible: hovered
              ToolTip.text: "Uncheck all topics for the current model (stored as overrides)"
              ToolTip.delay: 400
              onClicked: bridgeManager.uncheckAllCurrentModel()
            }
          }

          CheckBox {
            text: "Include checked topics from all models"
            font.pixelSize: 10; padding: 2
            checked: bridgeManager.includeAllModels
            onToggled: bridgeManager.setIncludeAllModels(checked)
            ToolTip.visible: hovered
            ToolTip.text: "Union of checked topics across every model you've curated in this world"
            ToolTip.delay: 400
          }

          Rectangle {
            Layout.fillWidth: true
            implicitHeight: bridgeManager.bridgeCommand.length > 0
                              ? Math.min(cmdLabel.implicitHeight + 10, 140) : 36
            color: bridgeManager.bridgeCommand.length > 0 ? "#f1f8e9" : "#fafafa"
            radius: 3
            border.color: bridgeManager.bridgeCommand.length > 0 ? "#a5d6a7" : "#e0e0e0"
            border.width: 1
            clip: true

            Flickable {
              anchors { fill: parent; margins: 5 }
              contentHeight: cmdLabel.implicitHeight
              clip: true
              visible: bridgeManager.bridgeCommand.length > 0

              Label {
                id: cmdLabel
                width: parent.width
                text: bridgeManager.bridgeCommandDisplay
                font.pixelSize: 10; font.family: "monospace"
                color: "#1b5e20"; wrapMode: Text.Wrap
              }
            }

            Label {
              anchors.centerIn: parent
              visible: bridgeManager.bridgeCommand.length === 0
              text: "No topics checked. Select a model or check topics below."
              font.pixelSize: 10; font.italic: true; color: "#9e9e9e"
            }
          }

          Label {
            visible: bridgeManager.missingTopicsWarning.length > 0
            text: "⚠ " + bridgeManager.missingTopicsWarning
            font.pixelSize: 10; color: "#bf360c"
            wrapMode: Text.Wrap; Layout.fillWidth: true
          }
        }
      }

      // ── 7. Heuristic suggestions ──────────────────────────────────
      // When ECM active: show only non-EcmConfirmed entries (collapsed).
      // When ECM unavailable: show all associated topics (expanded).
      Rectangle {
        id: assocCard
        Layout.leftMargin: 10; Layout.rightMargin: 10
        Layout.fillWidth: true
        implicitHeight: assocCol.implicitHeight + 16
        color: "#fafafa"
        border.color: "#e0e0e0"; border.width: 1
        radius: 4

        // Filter out ECM-confirmed entries when ECM is available; those are
        // already shown with checkboxes in the sensorTree section above.
        property var displayTopics: {
          var list = bridgeManager.associatedTopics;
          if (!list || !list.length) return [];
          if (!bridgeManager.ecmAvailable) return list;
          return list.filter(function(e) { return e.category !== "EcmConfirmed"; });
        }

        visible: displayTopics.length > 0

        // Collapsed when ECM is active (secondary information); expanded otherwise.
        property bool expanded: !bridgeManager.ecmAvailable

        ColumnLayout {
          id: assocCol
          anchors {
            top: parent.top; left: parent.left; right: parent.right
            topMargin: 8; leftMargin: 8; rightMargin: 8
          }
          spacing: 4

          Item {
            Layout.fillWidth: true
            implicitHeight: assocHeader.implicitHeight + 4

            Label {
              id: assocHeader
              text: (assocCard.expanded ? "▼" : "▶") +
                    "  Heuristic suggestions (" +
                    assocCard.displayTopics.length + ")"
              font.bold: true; font.pixelSize: 12; color: "#1b5e20"
            }
            MouseArea {
              anchors.fill: parent
              cursorShape: Qt.PointingHandCursor
              onClicked: assocCard.expanded = !assocCard.expanded
            }
          }

          RowLayout {
            visible: assocCard.expanded
            Layout.fillWidth: true

            Label {
              text: bridgeManager.ecmAvailable
                    ? "Topics matched by topic-name heuristic (not ECM-confirmed)."
                    : "Topics matched by topic-name heuristic. ECM not available."
              font.pixelSize: 9; font.italic: true; color: "#616161"
              wrapMode: Text.Wrap; Layout.fillWidth: true
            }

            Button {
              text: "Check all"; font.pixelSize: 10
              implicitWidth: 70; implicitHeight: 22
              onClicked: bridgeManager.checkAllAssociated()
            }
          }

          ListView {
            visible: assocCard.expanded
            Layout.fillWidth: true
            implicitHeight: Math.min(count * 30, 240)
            clip: true
            interactive: true
            ScrollBar.vertical: ScrollBar { policy: ScrollBar.AsNeeded }

            model: assocCard.displayTopics
            delegate: TopicRow { entry: modelData; checkable: true }
          }
        }
      }

      // ── 8. Additional bridgeable topics ───────────────────────────
      // Bridgeable topics that could not be confidently associated with
      // the selected model. Previously labelled "Bridgeable but unassigned".
      Rectangle {
        id: unassignedCard
        Layout.leftMargin: 10; Layout.rightMargin: 10
        Layout.fillWidth: true
        implicitHeight: unassnCol.implicitHeight + 16
        color: "#fffde7"
        border.color: "#ffe082"; border.width: 1
        radius: 4
        visible: bridgeManager.unassignedTopics.length > 0

        property bool expanded: bridgeManager.selectedModel.length === 0

        ColumnLayout {
          id: unassnCol
          anchors {
            top: parent.top; left: parent.left; right: parent.right
            topMargin: 8; leftMargin: 8; rightMargin: 8
          }
          spacing: 4

          Item {
            Layout.fillWidth: true
            implicitHeight: unassnHeader.implicitHeight + 4

            Label {
              id: unassnHeader
              text: (unassignedCard.expanded ? "▼" : "▶") +
                    "  Additional bridgeable topics (" +
                    bridgeManager.unassignedTopics.length + ")"
              font.bold: true; font.pixelSize: 12; color: "#e65100"
            }
            MouseArea {
              anchors.fill: parent
              cursorShape: Qt.PointingHandCursor
              onClicked: unassignedCard.expanded = !unassignedCard.expanded
            }
          }

          Label {
            visible: unassignedCard.expanded
            text: "Bridgeable topics not linked to sensors in the selected model. " +
                  "Generic topics (/clock, /scan, …) appear here by design."
            font.pixelSize: 10; font.italic: true
            color: "#5d4037"; wrapMode: Text.Wrap; Layout.fillWidth: true
          }

          ListView {
            visible: unassignedCard.expanded
            Layout.fillWidth: true
            implicitHeight: Math.min(count * 30, 240)
            clip: true; interactive: true
            ScrollBar.vertical: ScrollBar { policy: ScrollBar.AsNeeded }

            model: bridgeManager.unassignedTopics
            delegate: TopicRow { entry: modelData; checkable: true }
          }
        }
      }

      // ── 9. Unsupported / debug topics (collapsed) ─────────────────
      Rectangle {
        id: unsupCard
        Layout.leftMargin: 10; Layout.rightMargin: 10
        Layout.fillWidth: true
        implicitHeight: unsupCol.implicitHeight + 16
        color: "#fafafa"
        border.color: "#e0e0e0"; border.width: 1
        radius: 4
        visible: bridgeManager.unsupportedTopics.length > 0

        property bool expanded: false

        ColumnLayout {
          id: unsupCol
          anchors {
            top: parent.top; left: parent.left; right: parent.right
            topMargin: 8; leftMargin: 8; rightMargin: 8
          }
          spacing: 4

          Item {
            Layout.fillWidth: true
            implicitHeight: unsupHeader.implicitHeight + 4

            Label {
              id: unsupHeader
              text: (unsupCard.expanded ? "▼" : "▶") +
                    "  Unsupported / debug topics (" +
                    bridgeManager.unsupportedTopics.length + ")"
              font.bold: true; font.pixelSize: 12; color: "#757575"
            }
            MouseArea {
              anchors.fill: parent
              cursorShape: Qt.PointingHandCursor
              onClicked: unsupCard.expanded = !unsupCard.expanded
            }
          }

          ListView {
            visible: unsupCard.expanded
            Layout.fillWidth: true
            implicitHeight: Math.min(count * 30, 200)
            clip: true; interactive: true
            ScrollBar.vertical: ScrollBar { policy: ScrollBar.AsNeeded }

            model: bridgeManager.unsupportedTopics
            delegate: TopicRow { entry: modelData; checkable: false }
          }
        }
      }

      // ── 10. Empty state ────────────────────────────────────────────
      Rectangle {
        Layout.leftMargin: 10; Layout.rightMargin: 10
        Layout.fillWidth: true
        implicitHeight: emptyLabel.implicitHeight + 20
        color: "#f5f5f5"; radius: 4
        visible: !bridgeManager.busy &&
                 bridgeManager.worldName.length === 0 &&
                 bridgeManager.unsupportedTopics.length === 0 &&
                 !bridgeManager.hasBridgeableTopics

        Label {
          id: emptyLabel
          anchors {
            top: parent.top; left: parent.left; right: parent.right
            topMargin: 10; leftMargin: 10; rightMargin: 10
          }
          text: "Press Refresh to discover Gazebo worlds and topics.\n" +
                "Make sure gz sim is running."
          font.pixelSize: 12; color: "#9e9e9e"
          wrapMode: Text.Wrap; horizontalAlignment: Text.AlignHCenter
        }
      }

      Item { implicitHeight: 8 }
    }
  }
}

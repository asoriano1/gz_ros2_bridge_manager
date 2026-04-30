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

  // Show models with no ECM sensors (hidden by default).
  property bool showModelsWithoutSensors: false

  // Computed from C++ modelCards, filtered by showModelsWithoutSensors.
  property var visibleCards: {
    if (showModelsWithoutSensors) return bridgeManager.modelCards
    return bridgeManager.modelCards.filter(function(c) { return c.ecmSensorCount > 0 })
  }

  // ---- JS helpers --------------------------------------------------------

  // New matchSourceName() strings are already user-friendly — pass through.
  function matchSourceLabel(src) { return src }

  function matchSourceColor(src) {
    if (src === "ECM exact")     return "#0d47a1";
    if (src === "ECM prefix")    return "#1565c0";
    if (src === "ECM path")      return "#1976d2";
    if (src === "Name match")    return "#e65100";
    if (src === "Type fallback") return "#f57c00";
    if (src === "Unresolved")    return "#bf360c";
    return "#757575";
  }

  // ---- TopicRow: used by Additional and Unsupported sections -------------
  component TopicRow: Rectangle {
    id: rowRoot
    property var    entry
    property string modelName: ""
    property bool   checkable: true

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
        onToggled: {
          if (rowRoot.modelName.length > 0)
            bridgeManager.setTopicChecked(rowRoot.modelName, entry.topic, checked)
          else
            bridgeManager.setAdditionalTopicChecked(entry.topic, checked)
        }
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
        ToolTip.text: entry.warning && entry.warning.length > 0
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
          return t && t.length > 0 ? t.replace("gz.msgs.", "") : "?"
        }
        font.pixelSize: 10; font.family: "monospace"
        color: "#1565c0"
        elide: Text.ElideRight
        Layout.preferredWidth: 90
      }

      Label {
        text: entry.confidence || ""
        font.pixelSize: 9; font.italic: true
        color: "#757575"
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
          text: "All models"
          font.pixelSize: 10; padding: 4
          checked: root.showModelsWithoutSensors
          onToggled: root.showModelsWithoutSensors = checked
          ToolTip.visible: hovered
          ToolTip.text: "Show models with no detected ECM sensors"
          ToolTip.delay: 400
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
        implicitHeight: statusRow.implicitHeight + 14
        color: bridgeManager.worldName.length > 0 ? "#e8f5e9" : "#fce4ec"
        radius: 4

        RowLayout {
          id: statusRow
          anchors {
            left: parent.left; right: parent.right
            verticalCenter: parent.verticalCenter
            leftMargin: 8; rightMargin: 8
          }
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

      // ── 3. Global warnings ────────────────────────────────────────
      Rectangle {
        Layout.leftMargin: 10; Layout.rightMargin: 10
        Layout.fillWidth: true
        implicitHeight: globalWarnLabel.implicitHeight + 12
        color: "#fff3e0"; border.color: "#ef6c00"; border.width: 1; radius: 4
        visible: bridgeManager.warnings.length > 0

        Label {
          id: globalWarnLabel
          anchors {
            top: parent.top; left: parent.left; right: parent.right
            topMargin: 6; leftMargin: 8; rightMargin: 8
          }
          text: "⚠ " + bridgeManager.warnings.join("\n⚠ ")
          font.pixelSize: 10; color: "#bf360c"; wrapMode: Text.Wrap
        }
      }

      // ── 4. Model accordion cards ──────────────────────────────────
      Repeater {
        model: root.visibleCards

        delegate: Rectangle {
          id: modelCard
          Layout.leftMargin: 10; Layout.rightMargin: 10
          Layout.fillWidth: true
          implicitHeight: modelCardCol.implicitHeight + 16
          color: "#fafafa"
          border.color: "#e0e0e0"; border.width: 1
          radius: 4

          // Save outer modelData before inner Repeaters shadow it.
          property var  cardData: modelData
          property bool expanded: false

          ColumnLayout {
            id: modelCardCol
            anchors {
              top: parent.top; left: parent.left; right: parent.right
              topMargin: 8; leftMargin: 8; rightMargin: 8
            }
            spacing: 4

            // Card header — click anywhere to expand / collapse.
            Item {
              Layout.fillWidth: true
              implicitHeight: cardHeaderRow.implicitHeight + 4

              RowLayout {
                id: cardHeaderRow
                anchors { left: parent.left; right: parent.right }
                spacing: 6

                Label {
                  text: (modelCard.expanded ? "▼" : "▶") + "  " +
                        modelCard.cardData.modelName
                  font.bold: true; font.pixelSize: 12; color: "#212121"
                  Layout.fillWidth: true; elide: Text.ElideRight
                }

                // ECM dot — green if sensors found, grey otherwise.
                Rectangle {
                  width: 8; height: 8; radius: 4
                  color: modelCard.cardData.ecmAvailable ? "#43a047" : "#bdbdbd"
                  ToolTip.visible: dotHover.containsMouse
                  ToolTip.text: modelCard.cardData.ecmAvailable
                                ? modelCard.cardData.ecmSensorCount + " ECM sensor(s)"
                                : "No ECM sensors detected"
                  ToolTip.delay: 400
                  MouseArea {
                    id: dotHover; anchors.fill: parent
                    hoverEnabled: true; acceptedButtons: Qt.NoButton
                  }
                }

                Label {
                  text: {
                    var sel = modelCard.cardData.selectedTopicCount
                    return sel + " selected"
                  }
                  font.pixelSize: 10; color: "#616161"
                }

                Button {
                  text: "Reset"; font.pixelSize: 9
                  implicitWidth: 48; implicitHeight: 22
                  ToolTip.visible: hovered
                  ToolTip.text: "Reset this model's selections to ECM defaults"
                  ToolTip.delay: 400
                  onClicked: bridgeManager.resetModelSelection(modelCard.cardData.modelName)
                }
              }

              MouseArea {
                anchors.fill: parent
                cursorShape: Qt.PointingHandCursor
                onClicked: modelCard.expanded = !modelCard.expanded
              }
            }

            // ---- Expanded body: ECM sensor cards --------------------

            Repeater {
              model: modelCard.expanded && modelCard.cardData.ecmSensorCount > 0
                     ? modelCard.cardData.sensors : []

              delegate: Rectangle {
                id: sensorCard
                Layout.fillWidth: true
                implicitHeight: sensorCol.implicitHeight + 10
                radius: 3
                border.width: 1

                // Pin outer modelData before the inner topic Repeater.
                property var sensorData: modelData

                // Strong ECM match → green; weak match → orange; unresolved → yellow.
                property bool strongMatch: {
                  var ms = sensorCard.sensorData.matchSource
                  return ms === "ECM exact" || ms === "ECM prefix" || ms === "ECM path"
                }

                color:        strongMatch ? "#f1f8e9"
                              : (sensorData.resolved ? "#fff3e0" : "#fff8e1")
                border.color: strongMatch ? "#a5d6a7"
                              : (sensorData.resolved ? "#ffcc80" : "#ffe082")

                ColumnLayout {
                  id: sensorCol
                  anchors {
                    left: parent.left; right: parent.right; top: parent.top
                    leftMargin: 6; rightMargin: 6; topMargin: 4
                  }
                  spacing: 2

                  // Identity: link / sensor  [type]  [nested?]
                  RowLayout {
                    Layout.fillWidth: true; spacing: 4

                    Label {
                      text: sensorCard.sensorData.linkName + " / " +
                            sensorCard.sensorData.sensorName
                      font.pixelSize: 10; font.bold: true; font.family: "monospace"
                      color: "#1b5e20"; Layout.fillWidth: true; elide: Text.ElideRight
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

                  // Declared topic or fallback path
                  Label {
                    visible: sensorCard.sensorData.declaredTopic.length > 0
                    text: "topic: " + sensorCard.sensorData.declaredTopic
                    font.pixelSize: 9; font.family: "monospace"; color: "#424242"
                    Layout.fillWidth: true; elide: Text.ElideMiddle
                    ToolTip.visible: dtHover.containsMouse
                    ToolTip.text: sensorCard.sensorData.declaredTopic
                    ToolTip.delay: 300
                    MouseArea {
                      id: dtHover; anchors.fill: parent
                      hoverEnabled: true; acceptedButtons: Qt.NoButton
                    }
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
                    MouseArea {
                      id: fpHover; anchors.fill: parent
                      hoverEnabled: true; acceptedButtons: Qt.NoButton
                    }
                  }

                  // Column header row for per-topic list
                  RowLayout {
                    visible: sensorCard.sensorData.matchedTopicDetails.length > 0
                    Layout.fillWidth: true; Layout.leftMargin: 26; spacing: 3

                    Label {
                      text: "Gazebo topic"
                      font.pixelSize: 8; font.bold: true; color: "#9e9e9e"
                      Layout.fillWidth: true
                    }
                    Label {
                      text: "gz → ros2 type"
                      font.pixelSize: 8; font.bold: true; color: "#9e9e9e"
                      Layout.preferredWidth: 155
                    }
                  }

                  // Per-topic rows (checked state embedded in matchedTopicDetails)
                  Repeater {
                    model: sensorCard.sensorData.matchedTopicDetails

                    delegate: RowLayout {
                      id: tdRow
                      Layout.fillWidth: true; Layout.leftMargin: 8; spacing: 3

                      property string tdTopic: modelData.topic

                      CheckBox {
                        padding: 0
                        Layout.preferredWidth: 22
                        Layout.alignment: Qt.AlignVCenter
                        checked: modelData.checked
                        enabled: modelData.bridgeable
                        onToggled: bridgeManager.setTopicChecked(
                                       modelCard.cardData.modelName,
                                       tdRow.tdTopic, checked)
                      }

                      Label {
                        text: modelData.topic
                        font.pixelSize: 9; font.family: "monospace"; color: "#33691e"
                        Layout.fillWidth: true; elide: Text.ElideMiddle
                        ToolTip.visible: tdHover.containsMouse
                        ToolTip.text: modelData.topic; ToolTip.delay: 300
                        MouseArea {
                          id: tdHover; anchors.fill: parent
                          hoverEnabled: true; acceptedButtons: Qt.NoButton
                        }
                      }

                      Label {
                        text: modelData.gzType.replace("gz.msgs.", "")
                        font.pixelSize: 8; color: "#1565c0"
                        elide: Text.ElideRight; Layout.preferredWidth: 72
                      }

                      Label { text: "→"; font.pixelSize: 8; color: "#9e9e9e" }

                      Label {
                        text: {
                          var parts = modelData.ros2Type.split("/")
                          return parts.length > 0 ? parts[parts.length - 1] : modelData.ros2Type
                        }
                        font.pixelSize: 8; color: "#5d4037"
                        elide: Text.ElideRight; Layout.preferredWidth: 80
                      }
                    }
                  }

                  // Sensor warning — orange for weak match, red for truly unresolved.
                  Label {
                    visible: sensorCard.sensorData.warning.length > 0
                    text: "⚠ " + sensorCard.sensorData.warning
                    font.pixelSize: 9; font.italic: true
                    color: sensorCard.sensorData.resolved ? "#e65100" : "#bf360c"
                    wrapMode: Text.Wrap; Layout.fillWidth: true
                  }

                  Item { implicitHeight: 2 }
                }
              }
            }

            // ECM unavailable banner (inside card body, when model has no ECM data)
            Rectangle {
              Layout.fillWidth: true
              implicitHeight: ecmUnavailLabel.implicitHeight + 10
              color: "#fff8e1"; border.color: "#ffe082"; border.width: 1; radius: 3
              visible: modelCard.expanded && !modelCard.cardData.ecmAvailable

              Label {
                id: ecmUnavailLabel
                anchors {
                  left: parent.left; right: parent.right; verticalCenter: parent.verticalCenter
                  leftMargin: 6; rightMargin: 6
                }
                text: "⚠ No ECM sensors detected for this model."
                font.pixelSize: 10; color: "#f57f17"; wrapMode: Text.Wrap
              }
            }

          }  // modelCardCol
        }  // modelCard Rectangle
      }  // Repeater visibleCards

      // ── 5. Bridge command (collapsible, collapsed by default) ─────
      Rectangle {
        id: cmdCard
        Layout.leftMargin: 10; Layout.rightMargin: 10
        Layout.fillWidth: true
        implicitHeight: cmdCardCol.implicitHeight + 16
        color: bridgeManager.bridgeCommand.length > 0 ? "#e8f5e9" : "#f5f5f5"
        border.color: bridgeManager.bridgeCommand.length > 0 ? "#66bb6a" : "#bdbdbd"
        border.width: 1; radius: 4

        property bool expanded: false

        ColumnLayout {
          id: cmdCardCol
          anchors {
            top: parent.top; left: parent.left; right: parent.right
            topMargin: 8; leftMargin: 8; rightMargin: 8
          }
          spacing: 6

          // Header row (always visible) — click label area to toggle.
          Item {
            Layout.fillWidth: true
            implicitHeight: cmdHeaderRow.implicitHeight

            RowLayout {
              id: cmdHeaderRow
              anchors { left: parent.left; right: parent.right }
              spacing: 6

              Label {
                text: {
                  var n = bridgeManager.selectedBridgeTopicCount
                  var arrow = cmdCard.expanded ? "▼" : "▶"
                  return arrow + "  Bridge command  •  " + n + " topic" +
                         (n === 1 ? "" : "s") + " selected"
                }
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
            }

            MouseArea {
              anchors { left: parent.left; right: parent.right; top: parent.top; bottom: parent.bottom }
              onClicked: cmdCard.expanded = !cmdCard.expanded
            }
          }

          // Command display (shown when expanded)
          Rectangle {
            visible: cmdCard.expanded
            Layout.fillWidth: true
            implicitHeight: bridgeManager.bridgeCommand.length > 0
                              ? Math.min(cmdLabel.implicitHeight + 10, 140) : 36
            color: bridgeManager.bridgeCommand.length > 0 ? "#f1f8e9" : "#fafafa"
            radius: 3
            border.color: bridgeManager.bridgeCommand.length > 0 ? "#a5d6a7" : "#e0e0e0"
            border.width: 1; clip: true

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
              text: "No topics checked. Expand a model card above and check topics."
              font.pixelSize: 10; font.italic: true; color: "#9e9e9e"
              wrapMode: Text.Wrap; horizontalAlignment: Text.AlignHCenter
            }
          }
        }
      }

      // ── 6. Additional bridgeable topics (collapsed, unchecked by default) ─
      Rectangle {
        id: additionalCard
        Layout.leftMargin: 10; Layout.rightMargin: 10
        Layout.fillWidth: true
        implicitHeight: addCol.implicitHeight + 16
        color: "#fffde7"; border.color: "#ffe082"; border.width: 1; radius: 4
        visible: bridgeManager.additionalBridgeableTopics.length > 0

        property bool expanded: false

        ColumnLayout {
          id: addCol
          anchors {
            top: parent.top; left: parent.left; right: parent.right
            topMargin: 8; leftMargin: 8; rightMargin: 8
          }
          spacing: 4

          Item {
            Layout.fillWidth: true
            implicitHeight: addHeader.implicitHeight + 4

            Label {
              id: addHeader
              text: (additionalCard.expanded ? "▼" : "▶") +
                    "  Additional bridgeable topics (" +
                    bridgeManager.additionalBridgeableTopics.length + ")"
              font.bold: true; font.pixelSize: 12; color: "#e65100"
            }
            MouseArea {
              anchors.fill: parent; cursorShape: Qt.PointingHandCursor
              onClicked: additionalCard.expanded = !additionalCard.expanded
            }
          }

          Label {
            visible: additionalCard.expanded
            text: "Bridgeable topics not linked to any ECM sensor. All unchecked by default."
            font.pixelSize: 10; font.italic: true
            color: "#5d4037"; wrapMode: Text.Wrap; Layout.fillWidth: true
          }

          ListView {
            visible: additionalCard.expanded
            Layout.fillWidth: true
            implicitHeight: Math.min(count * 30, 240)
            clip: true; interactive: true
            ScrollBar.vertical: ScrollBar { policy: ScrollBar.AsNeeded }

            model: bridgeManager.additionalBridgeableTopics
            delegate: TopicRow {
              entry: modelData
              modelName: ""   // → setAdditionalTopicChecked
              checkable: true
            }
          }
        }
      }

      // ── 7. Unsupported / debug topics (collapsed) ─────────────────
      Rectangle {
        id: unsupCard
        Layout.leftMargin: 10; Layout.rightMargin: 10
        Layout.fillWidth: true
        implicitHeight: unsupCol.implicitHeight + 16
        color: "#fafafa"; border.color: "#e0e0e0"; border.width: 1; radius: 4
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
              anchors.fill: parent; cursorShape: Qt.PointingHandCursor
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

      // ── 8. Empty state ─────────────────────────────────────────────
      Rectangle {
        Layout.leftMargin: 10; Layout.rightMargin: 10
        Layout.fillWidth: true
        implicitHeight: emptyLabel.implicitHeight + 20
        color: "#f5f5f5"; radius: 4
        visible: !bridgeManager.busy &&
                 bridgeManager.worldName.length === 0 &&
                 bridgeManager.modelCards.length === 0 &&
                 bridgeManager.additionalBridgeableTopics.length === 0

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

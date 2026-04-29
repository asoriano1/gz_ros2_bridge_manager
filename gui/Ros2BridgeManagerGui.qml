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

  function topicCountLabel() {
    var assoc  = bridgeManager.associatedTopics.length
    var unassn = bridgeManager.unassignedTopics.length
    var unsup  = bridgeManager.unsupportedTopics.length
    return "Associated: " + assoc + "  •  Unassigned: " + unassn + "  •  Unsupported: " + unsup
  }

  // Reusable row delegate for one topic candidate.
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

      // ---- Header: title + Refresh + Auto-refresh ----
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
          font.pixelSize: 10
          padding: 4
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

      // ---- Status + last refresh ----
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
              font.pixelSize: 10
              color: "#558b2f"
            }
          }
        }
      }

      // ---- Model-gone warning ----
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

      // ---- Model selector ----
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
              text: "Reset"
              font.pixelSize: 10
              implicitWidth: 56; implicitHeight: 24
              ToolTip.visible: hovered
              ToolTip.text: "Reset this model's manual selections to heuristic defaults"
              ToolTip.delay: 400
              onClicked: bridgeManager.resetCurrentModelSelection()
            }
          }

          Label {
            text: "Selections are remembered per model during this session."
            font.pixelSize: 9; font.italic: true; color: "#9e9e9e"
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
        }
      }

      // ---- Bridge command ----
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
              Layout.fillWidth: true
              elide: Text.ElideRight
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
            font.pixelSize: 10
            padding: 2
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
              font.pixelSize: 10; font.italic: true
              color: "#9e9e9e"
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

      // ---- ECM sensor tree (shown only when ECM has confirmed data) ----
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

          Item {
            Layout.fillWidth: true
            implicitHeight: ecmHeader.implicitHeight + 4
            Label {
              id: ecmHeader
              text: (ecmCard.expanded ? "▼" : "▶") +
                    "  Detected sensors in selected model (" +
                    bridgeManager.sensorTree.length + ")"
              font.bold: true; font.pixelSize: 12; color: "#1b5e20"
            }
            MouseArea {
              anchors.fill: parent
              cursorShape: Qt.PointingHandCursor
              onClicked: ecmCard.expanded = !ecmCard.expanded
            }
          }

          Label {
            visible: ecmCard.expanded
            text: "ECM-confirmed hierarchy: sensor topics are matched directly from the " +
                  "Entity Component Manager. These are authoritative."
            font.pixelSize: 9; font.italic: true; color: "#2e7d32"
            wrapMode: Text.Wrap; Layout.fillWidth: true
          }

          // One row per sensor
          Repeater {
            model: ecmCard.expanded ? bridgeManager.sensorTree : []
            delegate: Rectangle {
              Layout.fillWidth: true
              implicitHeight: sensorRow.implicitHeight + 8
              color: modelData.resolved ? "#f1f8e9" : "#fff3e0"
              radius: 3
              border.color: modelData.resolved ? "#a5d6a7" : "#ffcc02"
              border.width: 1

              ColumnLayout {
                id: sensorRow
                anchors { left: parent.left; right: parent.right; verticalCenter: parent.verticalCenter
                          leftMargin: 6; rightMargin: 6; topMargin: 4; bottomMargin: 4 }
                spacing: 2

                RowLayout {
                  spacing: 6
                  Label {
                    text: modelData.linkName + " / " + modelData.sensorName
                    font.pixelSize: 10; font.bold: true; font.family: "monospace"
                    color: "#1b5e20"; Layout.fillWidth: true; elide: Text.ElideRight
                  }
                  Label {
                    text: modelData.sensorType
                    font.pixelSize: 9; color: "#388e3c"; font.italic: true
                  }
                  Label {
                    visible: !modelData.resolved
                    text: "⚠ no topic"
                    font.pixelSize: 9; color: "#e65100"
                  }
                }

                Label {
                  visible: modelData.resolved
                  text: modelData.matchedTopics.join("\n")
                  font.pixelSize: 9; font.family: "monospace"; color: "#33691e"
                  wrapMode: Text.Wrap; Layout.fillWidth: true
                }

                Label {
                  visible: modelData.warning.length > 0 && !modelData.resolved
                  text: modelData.warning
                  font.pixelSize: 9; font.italic: true; color: "#bf360c"
                  wrapMode: Text.Wrap; Layout.fillWidth: true
                }
              }
            }
          }
        }
      }

      // ---- Associated topics (ECM confirmed + heuristic) ----
      Rectangle {
        Layout.leftMargin: 10; Layout.rightMargin: 10
        Layout.fillWidth: true
        implicitHeight: assocCol.implicitHeight + 16
        color: "#fafafa"
        border.color: "#e0e0e0"; border.width: 1
        radius: 4
        visible: bridgeManager.associatedTopics.length > 0

        ColumnLayout {
          id: assocCol
          anchors {
            top: parent.top; left: parent.left; right: parent.right
            topMargin: 8; leftMargin: 8; rightMargin: 8
          }
          spacing: 4

          RowLayout {
            Layout.fillWidth: true
            Label {
              text: (bridgeManager.ecmAvailable
                       ? "Detected sensor topics (" : "Heuristic suggestions (") +
                    bridgeManager.associatedTopics.length + ")"
              font.bold: true; font.pixelSize: 12; color: "#1b5e20"
              Layout.fillWidth: true
            }
            Button {
              text: "Check all"; font.pixelSize: 10
              implicitWidth: 70; implicitHeight: 22
              onClicked: bridgeManager.checkAllAssociated()
            }
          }

          ListView {
            Layout.fillWidth: true
            implicitHeight: Math.min(count * 30, 240)
            clip: true
            interactive: true
            ScrollBar.vertical: ScrollBar { policy: ScrollBar.AsNeeded }

            model: bridgeManager.associatedTopics
            delegate: TopicRow {
              entry: modelData
              checkable: true
            }
          }
        }
      }

      // ---- Unassigned bridgeable ----
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
                    "  Bridgeable but unassigned (" +
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
            text: "These topics are bridgeable but could not be confidently associated with the selected model. " +
                  "Generic topics (/clock, /scan, …) are listed here on purpose."
            font.pixelSize: 10; font.italic: true
            color: "#5d4037"; wrapMode: Text.Wrap; Layout.fillWidth: true
          }

          ListView {
            visible: unassignedCard.expanded
            Layout.fillWidth: true
            implicitHeight: Math.min(count * 30, 240)
            clip: true
            interactive: true
            ScrollBar.vertical: ScrollBar { policy: ScrollBar.AsNeeded }

            model: bridgeManager.unassignedTopics
            delegate: TopicRow {
              entry: modelData
              checkable: true
            }
          }
        }
      }

      // ---- Unsupported (collapsed) ----
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
                    "  Unsupported types (" +
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
            clip: true
            interactive: true
            ScrollBar.vertical: ScrollBar { policy: ScrollBar.AsNeeded }

            model: bridgeManager.unsupportedTopics
            delegate: TopicRow {
              entry: modelData
              checkable: false
            }
          }
        }
      }

      // ---- Sensor hierarchy note ----
      Rectangle {
        Layout.leftMargin: 10; Layout.rightMargin: 10
        Layout.fillWidth: true
        implicitHeight: sensorNoteLabel.implicitHeight + 14
        color: bridgeManager.ecmAvailable ? "#e8f5e9" : "#fff8e1"
        border.color: bridgeManager.ecmAvailable ? "#81c784" : "#ffe082"
        border.width: 1
        radius: 4

        Label {
          id: sensorNoteLabel
          anchors {
            top: parent.top; left: parent.left; right: parent.right
            topMargin: 7; leftMargin: 8; rightMargin: 8
          }
          text: "Sensor hierarchy: " + bridgeManager.sensorNote
          font.pixelSize: 10; font.italic: true
          color: bridgeManager.ecmAvailable ? "#1b5e20" : "#5d4037"
          wrapMode: Text.Wrap
        }
      }

      // ---- Empty state ----
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

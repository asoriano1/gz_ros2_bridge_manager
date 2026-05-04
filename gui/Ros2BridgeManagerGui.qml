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

  // Expansion state stored outside Repeater delegates so it survives
  // modelCards rebuilds triggered by checkbox toggles.
  property var expandedModels: ({})

  // Debug mode: off by default; shows source, fallback path, match warnings.
  property bool debugMode: false

  // Show models with no ECM sensors (hidden by default).
  property bool showModelsWithoutSensors: false

  // Filtered model card list for the accordion Repeater.
  property var visibleCards: {
    if (showModelsWithoutSensors) return bridgeManager.modelCards
    return bridgeManager.modelCards.filter(function(c) { return c.ecmSensorCount > 0 })
  }

  // Reassign the whole map object to trigger QML binding updates.
  function setModelExpanded(name, val) {
    var m = {}
    for (var k in expandedModels) m[k] = expandedModels[k]
    m[name] = val
    expandedModels = m
  }

  function typeMappingLabel(gzType, ros2Type) {
    var gz = gzType && gzType.length > 0 ? gzType.replace("gz.msgs.", "") : ""
    if (gz && ros2Type) return gz + " → " + ros2Type
    return ros2Type || gz || "?"
  }

  function matchSourceLabel(src) {
    if (src === "ECM exact")     return "ECM SensorTopic"
    if (src === "ECM prefix")    return "ECM SensorTopic"
    if (src === "Unresolved")    return "Unresolved"
    return src
  }

  function matchSourceColor(src) {
    if (src === "ECM exact")     return "#0d47a1"
    if (src === "ECM prefix")    return "#1565c0"
    if (src === "Unresolved")    return "#bf360c"
    return "#757575"
  }

  function bridgeStatusColor(status) {
    if (status === "Running") return "#1b5e20"
    if (status === "Restart required") return "#e65100"
    if (status === "Starting" || status === "Stopping") return "#1565c0"
    if (status === "Failed" || status === "Crashed") return "#b71c1c"
    if (status === "Stopped") return "#2e7d32"
    if (status === "Exited") return "#424242"
    return "#757575"
  }

  // ---- TopicRow: used by Additional and Unsupported sections only ---------
  component TopicRow: Rectangle {
    id: rowRoot
    property var    entry
    property string modelName: ""
    property bool   checkable: true

    width: ListView.view ? ListView.view.width : implicitWidth
    height: 28
    color: index % 2 === 0 ? "#ffffff" : "#f5f5f5"

    RowLayout {
      anchors { fill: parent; leftMargin: 4; rightMargin: 4 }
      spacing: 4

      CheckBox {
        Layout.alignment: Qt.AlignVCenter
        Layout.preferredWidth: 22; padding: 0
        checked: entry.checked
        enabled: rowRoot.checkable && entry.bridgeable
        onToggled: {
          if (rowRoot.modelName.length > 0)
            bridgeManager.setTopicChecked(rowRoot.modelName, entry.topic, checked)
          else
            bridgeManager.setAdditionalTopicChecked(entry.topic, checked)
        }
      }

      Label {
        text: entry.topic
        font.pixelSize: 10; font.family: "monospace"
        elide: Text.ElideMiddle; Layout.fillWidth: true
        ToolTip.visible: th.containsMouse; ToolTip.text: entry.topic; ToolTip.delay: 400
        MouseArea { id: th; anchors.fill: parent; hoverEnabled: true; acceptedButtons: Qt.NoButton }
      }

      Label {
        text: root.typeMappingLabel(entry.gzType, entry.ros2Type)
        font.pixelSize: 9; color: "#5d4037"
        elide: Text.ElideRight; Layout.preferredWidth: 130
        ToolTip.visible: typeHover.containsMouse; ToolTip.text: text; ToolTip.delay: 300
        MouseArea { id: typeHover; anchors.fill: parent; hoverEnabled: true; acceptedButtons: Qt.NoButton }
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

        Item { Layout.fillWidth: true }

        CheckBox {
          text: "Debug/details"
          font.pixelSize: 10; padding: 4
          checked: root.debugMode
          onToggled: root.debugMode = checked
          ToolTip.visible: hovered
          ToolTip.text: "Show match source, declared topic, fallback path, and warnings"
          ToolTip.delay: 400
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
            running: bridgeManager.busy; visible: bridgeManager.busy
            width: 16; height: 16
          }

          Label {
            text: bridgeManager.statusText; font.pixelSize: 11
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
        implicitHeight: gwLabel.implicitHeight + 12
        color: "#fff3e0"; border.color: "#ef6c00"; border.width: 1; radius: 4
        visible: bridgeManager.warnings.length > 0

        Label {
          id: gwLabel
          anchors { top: parent.top; left: parent.left; right: parent.right; topMargin: 6; leftMargin: 8; rightMargin: 8 }
          text: "⚠ " + bridgeManager.warnings.join("\n⚠ ")
          font.pixelSize: 10; color: "#bf360c"; wrapMode: Text.Wrap
        }
      }

      // ── 4. Model accordion cards ──────────────────────────────────
      Label {
        Layout.leftMargin: 10
        Layout.rightMargin: 10
        Layout.fillWidth: true
        text: "Models:"
        font.pixelSize: 10
        font.bold: true
        color: "#616161"
      }

      Rectangle {
        Layout.leftMargin: 10; Layout.rightMargin: 10
        Layout.fillWidth: true
        implicitHeight: emptyModelsLabel.implicitHeight + 18
        color: "#f5f5f5"
        radius: 4
        visible: !bridgeManager.busy &&
                 bridgeManager.worldName.length > 0 &&
                 root.visibleCards.length === 0

        Label {
          id: emptyModelsLabel
          anchors {
            left: parent.left
            right: parent.right
            verticalCenter: parent.verticalCenter
            leftMargin: 10
            rightMargin: 10
          }
          text: "No models with sensors detected. Try Refresh or enable All models."
          font.pixelSize: 11
          color: "#757575"
          wrapMode: Text.Wrap
          horizontalAlignment: Text.AlignHCenter
        }
      }

      Repeater {
        model: root.visibleCards

        delegate: Rectangle {
          id: modelCard
          Layout.leftMargin: 10; Layout.rightMargin: 10
          Layout.fillWidth: true
          implicitHeight: mcCol.implicitHeight + 16
          color: "#fafafa"; border.color: "#e0e0e0"; border.width: 1; radius: 4

          // Pin modelData and read expansion from persistent root map.
          property var  cardData: modelData
          property bool expanded: root.expandedModels[cardData.modelName] === true

          ColumnLayout {
            id: mcCol
            anchors { top: parent.top; left: parent.left; right: parent.right; topMargin: 8; leftMargin: 8; rightMargin: 8 }
            spacing: 4

            // ── Card header ───────────────────────────────────────────
            RowLayout {
              Layout.fillWidth: true; spacing: 6

              // Only the arrow+name Label triggers expand/collapse;
              // Reset button and ECM dot handle their own clicks.
              Label {
                text: (modelCard.expanded ? "▼" : "▶") + "  " + modelCard.cardData.modelName
                font.bold: true; font.pixelSize: 12; color: "#212121"
                Layout.fillWidth: true; elide: Text.ElideRight

                MouseArea {
                  anchors.fill: parent; cursorShape: Qt.PointingHandCursor
                  onClicked: root.setModelExpanded(
                      modelCard.cardData.modelName, !modelCard.expanded)
                }
              }

              Rectangle {
                width: 8; height: 8; radius: 4
                color: modelCard.cardData.ecmAvailable ? "#43a047" : "#bdbdbd"
                ToolTip.visible: dotHov.containsMouse
                ToolTip.text: modelCard.cardData.ecmAvailable
                              ? modelCard.cardData.ecmSensorCount + " ECM sensor(s)"
                              : "No ECM sensors"
                ToolTip.delay: 400
                MouseArea { id: dotHov; anchors.fill: parent; hoverEnabled: true; acceptedButtons: Qt.NoButton }
              }

              Label {
                text: modelCard.cardData.selectedTopicCount + " selected"
                font.pixelSize: 10; color: "#616161"
              }

              Button {
                text: "Reset"; font.pixelSize: 9
                implicitWidth: 48; implicitHeight: 22
                ToolTip.visible: hovered; ToolTip.delay: 400
                ToolTip.text: "Reset topic selections to ECM defaults"
                onClicked: bridgeManager.resetModelSelection(modelCard.cardData.modelName)
              }
            }

            // ── Expanded body: compact sensor+topic rows ──────────────
            //
            // One ColumnLayout per sensor; inside it: one RowLayout per
            // matched topic, plus an unresolved row and optional debug block.
            Repeater {
              model: modelCard.expanded && modelCard.cardData.ecmSensorCount > 0
                     ? modelCard.cardData.sensors : []

              delegate: ColumnLayout {
                id: sensorDel
                property var sensorD: modelData
                // Capture sensor index before the inner Repeater overrides `index`.
                property int sensorIdx: index
                Layout.fillWidth: true
                spacing: 0

                // ── Compact topic rows ──────────────────────────────
                Repeater {
                  model: sensorDel.sensorD.matchedTopicDetails

                  delegate: RowLayout {
                    Layout.fillWidth: true; Layout.preferredHeight: 24; spacing: 4

                    CheckBox {
                      padding: 0; Layout.preferredWidth: 22
                      Layout.alignment: Qt.AlignVCenter
                      checked: modelData.checked; enabled: modelData.bridgeable
                      onToggled: bridgeManager.setTopicChecked(
                                     modelCard.cardData.modelName, modelData.topic, checked)
                    }

                    Label {
                      text: modelData.topic
                      font.pixelSize: 9; font.family: "monospace"; color: "#33691e"
                      Layout.fillWidth: true; elide: Text.ElideMiddle
                      ToolTip.visible: tpHov.containsMouse; ToolTip.text: modelData.topic; ToolTip.delay: 300
                      MouseArea { id: tpHov; anchors.fill: parent; hoverEnabled: true; acceptedButtons: Qt.NoButton }
                    }

                    Label {
                      text: root.typeMappingLabel(modelData.gzType, modelData.ros2Type)
                      font.pixelSize: 8; color: "#5d4037"
                      elide: Text.ElideRight; Layout.preferredWidth: 190
                      ToolTip.visible: typeMapHover.containsMouse
                      ToolTip.text: text
                      ToolTip.delay: 300
                      MouseArea {
                        id: typeMapHover
                        anchors.fill: parent
                        hoverEnabled: true
                        acceptedButtons: Qt.NoButton
                      }
                    }
                  }
                }  // topic Repeater

                // ── Unresolved row (sensor has no matched topics) ─────
                RowLayout {
                  visible: !sensorDel.sensorD.resolved &&
                           sensorDel.sensorD.matchedTopicDetails.length === 0
                  Layout.fillWidth: true; Layout.preferredHeight: 22; spacing: 4

                  Label { text: "⚠"; font.pixelSize: 10; color: "#bf360c"; Layout.preferredWidth: 14 }
                  Label {
                    text: sensorDel.sensorD.sensorName
                    font.pixelSize: 9; font.bold: true; font.family: "monospace"; color: "#bf360c"
                    Layout.preferredWidth: 112; elide: Text.ElideRight
                  }
                  Label {
                    text: sensorDel.sensorD.warning.length > 0
                          ? sensorDel.sensorD.warning
                          : "No matching topic found."
                    font.pixelSize: 9; font.italic: true; color: "#757575"
                    Layout.fillWidth: true
                    elide: Text.ElideRight
                  }
                }

                // ── Debug block (visible when Debug mode is on) ──────
                ColumnLayout {
                  visible: root.debugMode
                  Layout.fillWidth: true; Layout.leftMargin: 30; spacing: 1

                  Label {
                    text: "source: " + root.matchSourceLabel(sensorDel.sensorD.matchSource)
                    font.pixelSize: 8; font.italic: true
                    color: root.matchSourceColor(sensorDel.sensorD.matchSource)
                  }
                  Label {
                    text: "sensor type: " + sensorDel.sensorD.sensorType
                    font.pixelSize: 8; color: "#616161"
                  }
                  Label {
                    visible: sensorDel.sensorD.declaredTopic.length > 0
                    text: "TopicList contains Sensor Topic: " +
                          (sensorDel.sensorD.topicListed ? "yes" : "no")
                    font.pixelSize: 8; color: "#616161"
                  }
                  Label {
                    visible: sensorDel.sensorD.typeSource.length > 0
                    text: "type source: " + sensorDel.sensorD.typeSource
                    font.pixelSize: 8; color: "#616161"
                  }
                  Label {
                    visible: sensorDel.sensorD.topicInfoGzType.length > 0
                    text: "TopicInfo type: " + sensorDel.sensorD.topicInfoGzType
                    font.pixelSize: 8; font.family: "monospace"; color: "#424242"
                    Layout.fillWidth: true; elide: Text.ElideMiddle
                  }
                  Label {
                    visible: sensorDel.sensorD.inferredGzType.length > 0
                    text: "inferred type: " + sensorDel.sensorD.inferredGzType
                    font.pixelSize: 8; font.family: "monospace"; color: "#424242"
                    Layout.fillWidth: true; elide: Text.ElideMiddle
                  }
                  Label {
                    visible: sensorDel.sensorD.declaredTopic.length > 0
                    text: "Sensor Topic: " + sensorDel.sensorD.declaredTopic
                    font.pixelSize: 8; font.family: "monospace"; color: "#424242"
                    Layout.fillWidth: true; elide: Text.ElideMiddle
                    ToolTip.visible: dclHov.containsMouse; ToolTip.text: sensorDel.sensorD.declaredTopic; ToolTip.delay: 300
                    MouseArea { id: dclHov; anchors.fill: parent; hoverEnabled: true; acceptedButtons: Qt.NoButton }
                  }
                  Label {
                    visible: sensorDel.sensorD.declaredTopic.length === 0 &&
                             sensorDel.sensorD.fallbackPrefix.length > 0
                    text: "fallback: " + sensorDel.sensorD.fallbackPrefix
                    font.pixelSize: 8; font.family: "monospace"; color: "#616161"
                    Layout.fillWidth: true; elide: Text.ElideMiddle
                    ToolTip.visible: fpHov.containsMouse; ToolTip.text: sensorDel.sensorD.fallbackPrefix; ToolTip.delay: 300
                    MouseArea { id: fpHov; anchors.fill: parent; hoverEnabled: true; acceptedButtons: Qt.NoButton }
                  }
                  Label {
                    visible: sensorDel.sensorD.warning.length > 0
                    text: "⚠ " + sensorDel.sensorD.warning
                    font.pixelSize: 8; font.italic: true; wrapMode: Text.Wrap
                    color: sensorDel.sensorD.resolved ? "#e65100" : "#bf360c"
                    Layout.fillWidth: true
                  }
                }

                // Thin divider between sensors (not after the last one).
                Rectangle {
                  visible: sensorDel.sensorIdx < modelCard.cardData.ecmSensorCount - 1
                  Layout.fillWidth: true; height: 1; color: "#e8e8e8"
                }
              }  // sensorDel ColumnLayout
            }  // sensor Repeater

            // ECM unavailable banner
            Rectangle {
              Layout.fillWidth: true
              implicitHeight: ecuLabel.implicitHeight + 10
              color: "#fff8e1"; border.color: "#ffe082"; border.width: 1; radius: 3
              visible: modelCard.expanded && !modelCard.cardData.ecmAvailable

              Label {
                id: ecuLabel
                anchors { left: parent.left; right: parent.right; verticalCenter: parent.verticalCenter; leftMargin: 6; rightMargin: 6 }
                text: "⚠ No ECM sensors detected for this model."
                font.pixelSize: 10; color: "#f57f17"; wrapMode: Text.Wrap
              }
            }

          }  // mcCol
        }  // modelCard Rectangle
      }  // Repeater visibleCards

      // ── 5. Bridge runtime controls + command/output subsections ───
      Rectangle {
        id: cmdCard
        Layout.leftMargin: 10; Layout.rightMargin: 10
        Layout.fillWidth: true
        implicitHeight: cmdCol.implicitHeight + 16
        color: bridgeManager.bridgeCommand.length > 0 ? "#e8f5e9" : "#f5f5f5"
        border.color: bridgeManager.bridgeCommand.length > 0 ? "#66bb6a" : "#bdbdbd"
        border.width: 1; radius: 4

        property bool expanded: false
        property bool outputExpanded: false

        Connections {
          target: bridgeManager
          function onBridgeStatusTextChanged() {
            if (bridgeManager.bridgeStatusText === "Failed" ||
                bridgeManager.bridgeStatusText === "Crashed")
              cmdCard.outputExpanded = true
          }
        }

        ColumnLayout {
          id: cmdCol
          anchors { top: parent.top; left: parent.left; right: parent.right; topMargin: 8; leftMargin: 8; rightMargin: 8 }
          spacing: 6

          Rectangle {
            Layout.fillWidth: true
            implicitHeight: bridgeHeaderCol.implicitHeight + 12
            color: "#ffffff"
            radius: 3
            border.color: "#e0e0e0"
            border.width: 1

            ColumnLayout {
              id: bridgeHeaderCol
              anchors {
                top: parent.top
                left: parent.left
                right: parent.right
                topMargin: 6
                leftMargin: 8
                rightMargin: 8
              }
              spacing: 4

              RowLayout {
                Layout.fillWidth: true
                spacing: 8

                Label {
                  text: "Bridge runtime"
                  font.pixelSize: 12
                  font.bold: true
                  color: "#263238"
                }

                Label {
                  text: "Status: " + bridgeManager.bridgeStatusText
                  font.pixelSize: 10
                  font.bold: true
                  color: root.bridgeStatusColor(bridgeManager.bridgeStatusText)
                  Layout.fillWidth: true
                  wrapMode: Text.Wrap
                }
              }

              RowLayout {
                Layout.fillWidth: true
                spacing: 6

                Item { Layout.fillWidth: true }

                Button {
                  text: "Run"
                  font.pixelSize: 10
                  implicitHeight: 24
                  enabled: bridgeManager.selectedBridgeTopicCount > 0 &&
                           !bridgeManager.bridgeRunning &&
                           !bridgeManager.bridgeBusy
                  onClicked: bridgeManager.runBridge()
                }

                Button {
                  text: "Stop"
                  font.pixelSize: 10
                  implicitHeight: 24
                  enabled: bridgeManager.bridgeRunning || bridgeManager.bridgeBusy
                  onClicked: bridgeManager.stopBridge()
                }

                Button {
                  text: "Restart"
                  font.pixelSize: 10
                  implicitHeight: 24
                  visible: bridgeManager.bridgeRunning &&
                           bridgeManager.bridgeRestartRequired
                  enabled: visible && !bridgeManager.bridgeBusy
                  onClicked: bridgeManager.restartBridge()
                }
              }
            }
          }

          Rectangle {
            Layout.fillWidth: true
            implicitHeight: cmdSectionCol.implicitHeight + 10
            color: bridgeManager.bridgeCommand.length > 0 ? "#f8fbf7" : "#fafafa"
            radius: 3
            border.color: bridgeManager.bridgeCommand.length > 0 ? "#c5e1c5" : "#e0e0e0"
            border.width: 1

            ColumnLayout {
              id: cmdSectionCol
              anchors {
                top: parent.top
                left: parent.left
                right: parent.right
                topMargin: 5
                leftMargin: 6
                rightMargin: 6
              }
              spacing: 4

              Item {
                Layout.fillWidth: true
                implicitHeight: cmdHeaderRow.implicitHeight

                RowLayout {
                  id: cmdHeaderRow
                  anchors { left: parent.left; right: parent.right }
                  spacing: 6

                  Item {
                    Layout.fillWidth: true
                    implicitHeight: cmdTitle.implicitHeight

                    Label {
                      id: cmdTitle
                      anchors { left: parent.left; right: parent.right }
                      text: {
                        var n = bridgeManager.selectedBridgeTopicCount
                        return (cmdCard.expanded ? "▼" : "▶") + "  Bridge command  •  " +
                               n + " topic" + (n === 1 ? "" : "s") + " selected"
                      }
                      font.bold: true; font.pixelSize: 11
                      color: bridgeManager.bridgeCommand.length > 0 ? "#1b5e20" : "#757575"
                      elide: Text.ElideRight
                    }

                    MouseArea {
                      anchors.fill: parent
                      cursorShape: Qt.PointingHandCursor
                      onClicked: cmdCard.expanded = !cmdCard.expanded
                    }
                  }

                  Button {
                    text: "Copy"; font.pixelSize: 10
                    implicitWidth: 56; implicitHeight: 24
                    enabled: bridgeManager.bridgeCommand.length > 0
                    onClicked: bridgeManager.copyBridgeCommand()
                  }
                }
              }

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
                  contentHeight: cmdLabel.implicitHeight; clip: true
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

          Rectangle {
            Layout.fillWidth: true
            implicitHeight: outCol.implicitHeight + 10
            color: "#fafafa"
            radius: 3
            border.color: "#e0e0e0"
            border.width: 1

            ColumnLayout {
              id: outCol
              anchors {
                top: parent.top
                left: parent.left
                right: parent.right
                topMargin: 5
                leftMargin: 6
                rightMargin: 6
              }
              spacing: 4

              Item {
                Layout.fillWidth: true
                implicitHeight: outHeaderRow.implicitHeight

                RowLayout {
                  id: outHeaderRow
                  anchors {
                    left: parent.left
                    right: parent.right
                  }
                  spacing: 6

                  Item {
                    Layout.fillWidth: true
                    implicitHeight: outHeader.implicitHeight

                    Label {
                      id: outHeader
                      anchors {
                        left: parent.left
                        right: parent.right
                      }
                      text: (cmdCard.outputExpanded ? "▼" : "▶") + "  Bridge output"
                      font.bold: true
                      font.pixelSize: 11
                      color: "#424242"
                    }

                    MouseArea {
                      anchors.fill: parent
                      cursorShape: Qt.PointingHandCursor
                      onClicked: cmdCard.outputExpanded = !cmdCard.outputExpanded
                    }
                  }

                  Button {
                    text: "Clear output"
                    font.pixelSize: 10
                    implicitWidth: 88
                    implicitHeight: 24
                    enabled: bridgeManager.bridgeOutput.length > 0
                    onClicked: bridgeManager.clearBridgeOutput()
                  }
                }
              }

              Rectangle {
                visible: cmdCard.outputExpanded
                Layout.fillWidth: true
                implicitHeight: bridgeManager.bridgeOutput.length > 0
                                  ? Math.min(outLabel.implicitHeight + 10, 180) : 36
                color: "#ffffff"
                radius: 3
                border.color: "#e0e0e0"
                border.width: 1
                clip: true

                Flickable {
                  anchors.fill: parent
                  anchors.margins: 5
                  contentHeight: outLabel.implicitHeight
                  clip: true
                  visible: bridgeManager.bridgeOutput.length > 0

                  Label {
                    id: outLabel
                    width: parent.width
                    text: bridgeManager.bridgeOutput
                    font.pixelSize: 9
                    font.family: "monospace"
                    color: "#424242"
                    wrapMode: Text.Wrap
                  }
                }

                Label {
                  anchors.centerIn: parent
                  visible: bridgeManager.bridgeOutput.length === 0
                  text: "No bridge output yet."
                  font.pixelSize: 10
                  font.italic: true
                  color: "#9e9e9e"
                }
              }
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
          anchors { top: parent.top; left: parent.left; right: parent.right; topMargin: 8; leftMargin: 8; rightMargin: 8 }
          spacing: 4

          Item {
            Layout.fillWidth: true; implicitHeight: addHeader.implicitHeight + 4

            Label {
              id: addHeader
              text: (additionalCard.expanded ? "▼" : "▶") +
                    "  Additional bridgeable topics (" +
                    bridgeManager.additionalBridgeableTopics.length + ")"
              font.bold: true; font.pixelSize: 12; color: "#e65100"
            }
            MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor; onClicked: additionalCard.expanded = !additionalCard.expanded }
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
            implicitHeight: Math.min(count * 28, 240)
            clip: true; interactive: true
            ScrollBar.vertical: ScrollBar { policy: ScrollBar.AsNeeded }

            model: bridgeManager.additionalBridgeableTopics
            delegate: TopicRow { entry: modelData; modelName: ""; checkable: true }
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
          anchors { top: parent.top; left: parent.left; right: parent.right; topMargin: 8; leftMargin: 8; rightMargin: 8 }
          spacing: 4

          Item {
            Layout.fillWidth: true; implicitHeight: unsupHeader.implicitHeight + 4

            Label {
              id: unsupHeader
              text: (unsupCard.expanded ? "▼" : "▶") +
                    "  Unsupported / debug topics (" +
                    bridgeManager.unsupportedTopics.length + ")"
              font.bold: true; font.pixelSize: 12; color: "#757575"
            }
            MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor; onClicked: unsupCard.expanded = !unsupCard.expanded }
          }

          ListView {
            visible: unsupCard.expanded
            Layout.fillWidth: true
            implicitHeight: Math.min(count * 28, 200)
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
          anchors { top: parent.top; left: parent.left; right: parent.right; topMargin: 10; leftMargin: 10; rightMargin: 10 }
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

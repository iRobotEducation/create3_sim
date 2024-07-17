/*
 * Copyright 2021 Clearpath Robotics, Inc.
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */
 
import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Controls.Styles 1.4
import QtQuick.Layouts 1.3
import "qrc:/qml"

Rectangle 
{
  id: widgetRectangle
  color: "white"
  anchors.fill: parent
  focus: true
  Layout.minimumWidth: 400
  Layout.minimumHeight: 225

  Rectangle
  {
    id: create3ButtonsRectangle
    border.width: 2
    anchors.top: widgetRectangle.top
    anchors.left: widgetRectangle.left
    focus: true
    height: 175
    width: 400

    // Robot namespace input
    Label {
      id: namespaceLabel
      text: "Namespace:"
      Layout.fillWidth: true
      Layout.margins: 10
      anchors.top: create3ButtonsRectangle.top
      anchors.topMargin: 10
      anchors.left: create3ButtonsRectangle.left
      anchors.leftMargin: 10
    }

    TextField {
      id: nameField
      width: 175
      Layout.fillWidth: true
      Layout.margins: 10
      text: Create3Hmi.namespace
      placeholderText: qsTr("Robot namespace")
      anchors.top: namespaceLabel.bottom
      anchors.topMargin: 5
      anchors.left: create3ButtonsRectangle.left
      anchors.leftMargin: 10
      onEditingFinished: {
        Create3Hmi.SetNamespace(text)
      }
    }

    // Button inputs
    ToolButton {
      id: create3Button1
      anchors.verticalCenter: create3ButtonPower.verticalCenter
      anchors.right: create3ButtonPower.left
      anchors.rightMargin: 15
      checkable: true
      checked: true
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "images/One Dot.png"
        sourceSize.width: 36;
        sourceSize.height: 36;
      }
      onPressed: { Create3Hmi.OnCreate3Button(1); }
      onReleased: { Create3Hmi.OnCreate3Button(0); }
    }

    ToolButton {
      id: create3ButtonPower
      anchors.bottom: create3ButtonsRectangle.bottom
      anchors.bottomMargin: 15
      anchors.horizontalCenter: create3ButtonsRectangle.horizontalCenter
      checkable: true
      checked: true
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "images/Power.png"
        sourceSize.width: 72;
        sourceSize.height: 72;
      }
      onPressed: { Create3Hmi.OnCreate3Button(2); }
      onReleased: { Create3Hmi.OnCreate3Button(0); }
    }

    ToolButton {
      id: create3Button2
      anchors.verticalCenter: create3ButtonPower.verticalCenter
      anchors.left: create3ButtonPower.right
      anchors.leftMargin: 15
      checkable: true
      checked: true
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "images/Two Dots.png"
        sourceSize.width: 36;
        sourceSize.height: 36;
      }
      onPressed: { Create3Hmi.OnCreate3Button(3); }
      onReleased: { Create3Hmi.OnCreate3Button(0); }
    }
  }
}

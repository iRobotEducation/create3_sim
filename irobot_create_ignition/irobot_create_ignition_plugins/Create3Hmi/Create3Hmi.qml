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
  Layout.minimumHeight: 175

  Rectangle
  {
    id: create3ButtonsRectangle
    border.color: "black"
    border.width: 2
    anchors.top: widgetRectangle.top
    anchors.left: widgetRectangle.left
    focus: true
    height: 125
    width: 400

    // Buttons
    Label {
      id: create3ButtonsLabel
      text: "Create3"
      font.pixelSize: 22
      anchors.top: create3ButtonsRectangle.top
      anchors.topMargin: 10
      anchors.left: parent.left
      anchors.leftMargin: 10
    }

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
      anchors.top: create3ButtonsLabel.bottom
      anchors.topMargin: 0
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


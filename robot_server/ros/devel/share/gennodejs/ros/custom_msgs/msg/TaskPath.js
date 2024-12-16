// Auto-generated. Do not edit!

// (in-package custom_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class TaskPath {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robotName = null;
      this.currentWorkflowStep = null;
      this.path = null;
    }
    else {
      if (initObj.hasOwnProperty('robotName')) {
        this.robotName = initObj.robotName
      }
      else {
        this.robotName = '';
      }
      if (initObj.hasOwnProperty('currentWorkflowStep')) {
        this.currentWorkflowStep = initObj.currentWorkflowStep
      }
      else {
        this.currentWorkflowStep = '';
      }
      if (initObj.hasOwnProperty('path')) {
        this.path = initObj.path
      }
      else {
        this.path = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TaskPath
    // Serialize message field [robotName]
    bufferOffset = _serializer.string(obj.robotName, buffer, bufferOffset);
    // Serialize message field [currentWorkflowStep]
    bufferOffset = _serializer.string(obj.currentWorkflowStep, buffer, bufferOffset);
    // Serialize message field [path]
    // Serialize the length for message field [path]
    bufferOffset = _serializer.uint32(obj.path.length, buffer, bufferOffset);
    obj.path.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TaskPath
    let len;
    let data = new TaskPath(null);
    // Deserialize message field [robotName]
    data.robotName = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [currentWorkflowStep]
    data.currentWorkflowStep = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [path]
    // Deserialize array length for message field [path]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.path = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.path[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.robotName);
    length += _getByteLength(object.currentWorkflowStep);
    length += 24 * object.path.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'custom_msgs/TaskPath';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '970b44acb4bfcd0885f6e14494a46276';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string robotName
    string currentWorkflowStep
    geometry_msgs/Point[] path
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TaskPath(null);
    if (msg.robotName !== undefined) {
      resolved.robotName = msg.robotName;
    }
    else {
      resolved.robotName = ''
    }

    if (msg.currentWorkflowStep !== undefined) {
      resolved.currentWorkflowStep = msg.currentWorkflowStep;
    }
    else {
      resolved.currentWorkflowStep = ''
    }

    if (msg.path !== undefined) {
      resolved.path = new Array(msg.path.length);
      for (let i = 0; i < resolved.path.length; ++i) {
        resolved.path[i] = geometry_msgs.msg.Point.Resolve(msg.path[i]);
      }
    }
    else {
      resolved.path = []
    }

    return resolved;
    }
};

module.exports = TaskPath;

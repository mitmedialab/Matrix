#include "ofApp.h"

void ofApp::setup(void) {
  ofSetVerticalSync(true);
  ofSetWindowTitle("E256 - V1.0");

  ofSetLogLevel(OF_LOG_VERBOSE);
  /*
  OF_LOG_VERBOSE
  OF_LOG_NOTICE
  OF_LOG_WARNING
  OF_LOG_ERROR
  OF_LOG_FATAL_ERROR
  OF_LOG_SILENT
  */
  auto devicesInfo = SerialDeviceUtils::listDevices();
  ofLogNotice("ofApp::setup") << "Connected Devices: ";
  for (auto& SerialDevice: devicesInfo) ofLogNotice("ofApp::setup") << "\t" << SerialDevice;

  /*
  using SerialDevice::port;
  using SerialDevice::baudRate;
  using SerialDevice::dataBits;
  using SerialDevice::stopBits;
  using SerialDevice::timeout;
  using SerialDevice::isClearToSend;
  using SerialDevice::isDataSetReady;
  using SerialDevice::isRingIndicated;
  using SerialDevice::isCarrierDetected;
  using SerialDevice::isOpen;
  using SerialDevice::setDataTerminalReady;
  using SerialDevice::getPortName;
  */
  if (!devicesInfo.empty()) {
    bool success = serialDevice.setup(
      USB_PORT,
      BAUD_RATE
      //SerialDevice::DATA_BITS_EIGHT,
      //SerialDevice::PAR_NONE,
      //SerialDevice::STOP_ONE,
      //SerialDevice::FLOW_CTRL_HARDWARE
    );
    if (success) {
      serialDevice.registerAllEvents(this);
      ofLogNotice("ofApp::setup") << "Successfully setup: " << USB_PORT;
    }
    else {
      ofLogNotice("ofApp::setup") << "Unable to setup: " << USB_PORT;
    }
  }
  else {
    ofLogNotice("ofApp::setup") << "No devices connected!";
  }
  //sender.setup(HOST, UDP_OUTPUT_PORT); // OSC - UDP config
  //receiver.setup(UDP_INPUT_PORT); // SLIP-OSC via wifi

  setCalirationButton.addListener(this, &ofApp::E256_setCaliration);
  setTresholdSlider.addListener(this, &ofApp::E256_setTreshold);
  getRawDataToggle.addListener(this, &ofApp::E256_getRawDataStart);
  getInterpDataToggle.addListener(this, &ofApp::E256_getInterpDataStart);
  getBlobsToggle.addListener(this, &ofApp::E256_getBlobsStart);

  gui.setup("E256 - Parameters");
  gui.add(setCalirationButton.setup("Calibrate"));
  gui.add(setTresholdSlider.setup("Threshold", 10, 0, 100));
  gui.add(getRawDataToggle.setup("getRawData", true));
  gui.add(getInterpDataToggle.setup("getInterpData", false));
  gui.add(getBlobsToggle.setup("getBlobs", false));

  ofBackground(0);
  // 16 * 16
  for (int y=0; y<ROWS; y++) {
        for (int x=0; x<COLS; x++) {
            rawDataMesh.addVertex(ofPoint(x, y, 0));             // make a new vertex
            rawDataMesh.addColor(ofFloatColor(255, 255, 255));   // set vertex color to white
        }
    }
    for (int y=0; y<ROWS-1; y++) {
      for (int x=0; x<COLS-1; x++) {
        int i1 = x + y * COLS;           // 0, 1, 2, 3, 4
        int i2 = (x+1) + y * COLS;       // 1, 2, 3, 4,
        int i3 = x + (y+1) * COLS;       // 18, 19,
        int i4 = (x+1) + (y+1) * COLS;
        rawDataMesh.addTriangle(i1, i2, i4);
        rawDataMesh.addTriangle(i1, i3, i4);
    }
  }

  // 64 * 64
  for (int y=0; y<NEW_ROWS; y++) {
        for (int x=0; x<NEW_COLS; x++) {
            interpDataMesh.addVertex(ofPoint(x, y, 0));             // make a new vertex
            interpDataMesh.addColor(ofFloatColor(255, 255, 255));   // set vertex color to white
        }
    }
    for (int y=0; y<NEW_COLS-1; y++) {
      for (int x=0; x<NEW_COLS-1; x++) {
        int i1 = x + y * NEW_ROWS;           // 0, 1, 2, 3, 4
        int i2 = (x+1) + y * NEW_ROWS;       // 1, 2, 3, 4,
        int i3 = x + (y+1) * NEW_ROWS;       // 18, 19,
        int i4 = (x+1) + (y+1) * NEW_ROWS;
        interpDataMesh.addTriangle(i1, i2, i4);
        interpDataMesh.addTriangle(i1, i3, i4);
    }
  }

}

/////////////////////// SERIAL EVENT ///////////////////////
void ofApp::onSerialBuffer(const ofxIO::SerialBufferEventArgs& args) {

  if (getRawData){
    int offset = 12;
    std::copy(args.buffer().begin(), args.buffer().end(), inputFrameBuffer);
    //ofLogNotice("ofApp::onSerialBuffer") << "E256 - Serial message size : "<< message.OSCmessage.size();
    //ofLogNotice("ofApp::onSerialBuffer") << "E256 - Serial message : " << message.OSCmessage;

    for (int i=0; i<ROW_FRAME; i++){
      rawValues[i] = inputFrameBuffer[i + offset];
      //ofLogNotice("ofApp::onSerialBuffer") << "INDEX_" << i << " val_" << rawValues[i];
    }
    //////// Change vertices by the matrix sensors values
    for (int index=0; index<ROW_FRAME; index++) {
        ofPoint p = rawDataMesh.getVertex(index);    // Get the point coordinates
        p.z = rawValues[index];                      // Change the z-coordinates
        rawDataMesh.setVertex(index, p);             // Set the new coordinates
        rawDataMesh.setColor(index, ofColor(rawValues[index], 0, 255));    // Change vertex color
    }
    E256_getRawData();
  }

  if (getInterpData){
    int offset = 12;
    std::copy(args.buffer().begin(), args.buffer().end(), inputFrameBuffer);
    //ofLogNotice("ofApp::onSerialBuffer") << "E256 - Serial message size : "<< message.OSCmessage.size();
    //ofLogNotice("ofApp::onSerialBuffer") << "E256 - Serial message : " << message.OSCmessage;

    for (int i=0; i<NEW_FRAME; i++){
      interpValues[i] = inputFrameBuffer[i + offset];
      //ofLogNotice("ofApp::onSerialBuffer") << "INDEX_" << i << " val_" << interpValues[i];
    }
    //////// Change vertices by the matrix sensors values
    for (int index=0; index<NEW_FRAME; index++) {
        ofPoint p = interpDataMesh.getVertex(index);    // Get the point coordinates
        p.z = interpValues[index];                      // Change the z-coordinates
        interpDataMesh.setVertex(index, p);             // Set the new coordinates
        interpDataMesh.setColor(index, ofColor(interpValues[index], 0, 255));    // Change vertex color
    }
    E256_getInterpData();
  }

  if (getBlobs){
    message.OSCmessage = args.buffer().toString();
    int offset = 12;
    int stringOffset = 0;
    //ofLogNotice("ofApp::onSerialBuffer") << "E256 - Serial message size : "<< message.OSCmessage.size();
    //ofLogNotice("ofApp::onSerialBuffer") << "E256 - Serial message : " << message.OSCmessage;

    Poco::RegularExpression regex("/b(.){17}"); // GET X bytes after the "/b"
    Poco::RegularExpression::Match theMatch;

    /*
    for (size_t i=0; i<message.OSCmessage.size(); i++){
    ofLogNotice("ofApp::onSerialBuffer") << "INDEX_" << i << " val_" << ofToString(message.OSCmessage[i]);
    }
    */

    // Expand SLIP-OSC serial message to OSC messages
    // https://en.cppreference.com/w/cpp/regex
    while (regex.match(message.OSCmessage, stringOffset, theMatch)){
      std::string msg = std::string(message.OSCmessage, theMatch.offset, theMatch.length);
      ofxOscMessage oscMessage;
      oscMessage.setAddress("/b");
      oscMessage.addIntArg(msg[offset]);     // UID
      oscMessage.addIntArg(msg[offset + 1]); // alive
      oscMessage.addIntArg(msg[offset + 2]); // Xcentroide
      oscMessage.addIntArg(msg[offset + 3]); // Ycentroid
      oscMessage.addIntArg(msg[offset + 4]); // boxW
      oscMessage.addIntArg(msg[offset + 5]); // boxH
      oscMessage.addIntArg((uint8_t)msg[offset + 6]); // boxD
      blobs.push_back(oscMessage);
      stringOffset = theMatch.offset + theMatch.length;
    }
    E256_getBlobs();
  }
}

void ofApp::onSerialError(const ofxIO::SerialBufferErrorEventArgs& args) {
  message.exception = args.exception().displayText();
  ofLogNotice("ofApp::onSerialError") << "E256 - Serial ERROR : " << args.exception().displayText();
}

/////////////////////// UPDATE ///////////////////////
void ofApp::update(void) {

}

//////////////////////// DRAW ////////////////////////
void ofApp::draw(void) {
  ofBackground(0);
  gui.draw();

  std::stringstream dashboard;
  dashboard << "     Connected to : " << serialDevice.port() << std::endl;
  //dashboard << "SLIP-OSC-OUT port : " << UDP_OUTPUT_PORT << std::endl;
  //dashboard << " SLIP-OSC-IN port : " << UDP_INPUT_PORT << std::endl;
  dashboard << "              FPS : " << (int)ofGetFrameRate() << std::endl;
  ofDrawBitmapString(dashboard.str(), ofVec2f(20, 200)); // Draw the GUI menu


  //const int x = 0;  // X ofset
  //const int y = 0;  // Y ofset FIXME : dont afect the matrix graph
  const int SCALE_H = 50; // Scale 14
  const int SCALE_V = 50; // Scale 14
  const int BLOB_SCALE = 10;

  if (getRawData){
    ofPushMatrix();
    ofSetLineWidth(1);    // set line width to 1
    ofRotateDeg(30, 1, 0, 0);
    ofTranslate(ofGetWindowWidth()/3, ofGetWindowHeight()/8);
    ofScale(SCALE_H, SCALE_V, 1);
    rawDataMesh.drawWireframe(); // draws lines
    ofPopMatrix();
  }

  if (getInterpData){
    ofPushMatrix();
    ofSetLineWidth(1);    // set line width to 1
    ofRotateDeg(30, 1, 0, 0);
    ofTranslate(ofGetWindowWidth()/3, ofGetWindowHeight()/8);
    ofScale(SCALE_H/4, SCALE_V/4, 1);
    interpDataMesh.drawWireframe(); // draws lines
    ofPopMatrix();
  }

  if (getBlobs){
    ofPushMatrix();
    ofRotateDeg(30, 1, 0, 0);

    for (size_t index = 0; index < blobs.size(); ++index){
      if(blobs[index].getAddress() == "/b"){
        uint8_t blobID    = blobs[index].getArgAsInt(0) & 0xFF;
        uint8_t alive     = blobs[index].getArgAsInt(1) & 0xFF;
        float Xcentroid   = blobs[index].getArgAsInt(2) & 0xFF;
        float Ycentroid   = blobs[index].getArgAsInt(3) & 0xFF;
        uint8_t boxW      = blobs[index].getArgAsInt(4) & 0xFF;
        uint8_t boxH      = blobs[index].getArgAsInt(5) & 0xFF;
        uint8_t boxD      = blobs[index].getArgAsInt(6) & 0xFF;
        //ofLog(OF_LOG_VERBOSE,"E256_INPUT: UID:%d ALIVE:%d CX:%f CY:%f BW:%d BH:%d BD:%d",blobID, alive, Xcentroid, Ycentroid, boxW, boxH, boxD);

        Xcentroid = (Xcentroid / NEW_COLS) * ofGetWindowWidth();
        Ycentroid = (Ycentroid / NEW_ROWS) * ofGetWindowHeight();
        boxW = boxW * BLOB_SCALE;
        boxH = boxH * BLOB_SCALE;

        ofBoxPrimitive box;
        ofSetLineWidth(2);    // set line width to 1
        ofTranslate(ofGetWindowWidth()/4, ofGetWindowHeight()/8);
        ofScale(.6, .6, .6);
        ofSetColor(255);

        if (alive > 0){
          box.setMode(OF_PRIMITIVE_TRIANGLES);
          box.setResolution(1);
          box.set((float)boxW, (float)boxH, (float)boxD);
          box.setPosition(Xcentroid - boxW*0.5, Ycentroid - boxH*0.5, 0);
          box.drawWireframe();
          // box.draw();
          }
        else {
          blobs.erase(blobs.begin() + index);
        }
      }
    }
    blobs.clear();
    ofPopMatrix();
  }
}

// E256 matrix sensor - SET CALIBRATION
void ofApp::E256_setCaliration(void) {
  ofxOscMessage OSCmsg;
  OSCmsg.setAddress("/c"); // calibrate
  OSCmsg.addInt32Arg(10); // Set calibration cycles

  osc::OutboundPacketStream packet(requestBuffer, 1024);
  packet.Clear();
  packet << osc::BeginMessage(OSCmsg.getAddress().data());
  packet << OSCmsg.getArgAsInt32(0);
  packet << osc::EndMessage;
  serialDevice.send(ByteBuffer(packet.Data(), packet.Size()));
  ofLogNotice("ofApp::E256_setCaliration") << "E256 - Calibration requested : " << OSCmsg.getArgAsInt32(0);
}

// E256 matrix sensor - SET THRESHOLD
void ofApp::E256_setTreshold(int & tresholdValue) {
  ofxOscMessage OSCmsg;
  OSCmsg.setAddress("/t"); // threshold
  OSCmsg.addIntArg((int32_t)tresholdValue);

  osc::OutboundPacketStream packet(requestBuffer, 1024);
  packet.Clear();
  packet << osc::BeginMessage(OSCmsg.getAddress().data());
  packet << OSCmsg.getArgAsInt32(0);
  packet << osc::EndMessage;
  serialDevice.send(ByteBuffer(packet.Data(), packet.Size()));
  ofLogNotice("ofApp::E256_setTreshold") << "E256 - Threshold seted : " << OSCmsg.getArgAsInt32(0);
}

// E256 matrix sensor - MATRIX DATA REQUEST START
// 16*16 matrix row data request
void ofApp::E256_getRawDataStart(bool & val) {
  getRawData = val;
  E256_getRawData();
}
// E256 matrix sensor - BLOBS REQUEST START
void ofApp::E256_getInterpDataStart(bool & val) {
  getInterpData = val;
  E256_getInterpData();
}
// E256 matrix sensor - BLOBS REQUEST START
void ofApp::E256_getBlobsStart(bool & val) {
  getBlobs = val;
  E256_getBlobs();
}


// E256 matrix sensor - MATRIX DATA REQUEST
// 16*16 matrix row data request
void ofApp::E256_getRawData() {
  ofxOscMessage OSCmsg;
  OSCmsg.setAddress("/r"); //rowData
  osc::OutboundPacketStream packet(requestBuffer, 1024);
  packet.Clear();
  packet << osc::BeginMessage(OSCmsg.getAddress().data());
  packet << osc::EndMessage;
  serialDevice.send(ByteBuffer(packet.Data(), packet.Size()));
}

// E256 matrix sensor - MATRIX DATA REQUEST
// 64*64 matrix interpolated data request
void ofApp::E256_getInterpData() {
  ofxOscMessage OSCmsg;
  OSCmsg.setAddress("/i"); //rowData
  osc::OutboundPacketStream packet(requestBuffer, 1024);
  packet.Clear();
  packet << osc::BeginMessage(OSCmsg.getAddress().data());
  packet << osc::EndMessage;
  serialDevice.send(ByteBuffer(packet.Data(), packet.Size()));
}


// E256 matrix sensor - BLOBS REQUEST
void ofApp::E256_getBlobs() {
  ofxOscMessage OSCmsg;
  OSCmsg.setAddress("/b"); // Blobs
  osc::OutboundPacketStream packet(requestBuffer, 1024);
  packet.Clear();
  packet << osc::BeginMessage(OSCmsg.getAddress().data());
  packet << osc::EndMessage;
  serialDevice.send(ByteBuffer(packet.Data(), packet.Size()));
}

// E256 matrix sensor - Toggle full screen mode
void ofApp::keyPressed(int key) {
  switch(key) {
    case 'f':
    ofToggleFullscreen();
    break;
    default:
    break;
  }
}

void ofApp::exit(void) {
  setCalirationButton.removeListener(this, &ofApp::E256_setCaliration);
  setTresholdSlider.removeListener(this, &ofApp::E256_setTreshold);
  getRawDataToggle.removeListener(this, &ofApp::E256_getRawDataStart);
  getInterpDataToggle.removeListener(this, &ofApp::E256_getInterpDataStart);
  getBlobsToggle.removeListener(this, &ofApp::E256_getBlobsStart);
  serialDevice.unregisterAllEvents(this);
}

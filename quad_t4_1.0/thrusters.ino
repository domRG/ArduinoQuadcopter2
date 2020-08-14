float thrustLim[2] = {1000.0, 2000.0};

void setupThrusters(bool doCal){
  fR.attach(2);
  fL.attach(3);
  bR.attach(4);
  bL.attach(5);
  if(doCal){
    fR.writeMicroseconds((int)thrustLim[1]);
    fL.writeMicroseconds((int)thrustLim[1]);
    bR.writeMicroseconds((int)thrustLim[1]);
    bL.writeMicroseconds((int)thrustLim[1]);
    delay(1000);
  }
  fR.writeMicroseconds((int)thrustLim[0]);
  fL.writeMicroseconds((int)thrustLim[0]);
  bR.writeMicroseconds((int)thrustLim[0]);
  bL.writeMicroseconds((int)thrustLim[0]);
}

void updateThrusters(thrusterData_t* speeds){
  fR.writeMicroseconds((int)speeds->fR);
  fL.writeMicroseconds((int)speeds->fL);
  bR.writeMicroseconds((int)speeds->bR);
  bL.writeMicroseconds((int)speeds->bL);
}

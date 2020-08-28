

//----------------------------------------------------------------------------------------------------------------
// VOLUME O2
//----------------------------------------------------------------------------------------------------------------

int maxvolumeoxygen = 0;
int startupiterations = 10;

void FLOW_SENSOR_resetVolumeO2() {
  Volume_l_O2 = 0;
  totalFlow_O2 = 0;
  maxvolumeoxygen = 0;
}

void FLOW_SENSOR_updateVolumeO2(float flow_O2) { //flow = liter/min
  totalFlow_O2 += flow_O2;
  Volume_l_O2 = totalFlow_O2 * ((float)deltaT / 60000);
  float densitycorrection = 1;//density_air/density_o2;
  Volume_ml_O2 = (int)(Volume_l_O2 * 1000.0 / densitycorrection); // TODO: CHECK IF THIS WORKS! DENSITY CORRECTION
  if (Volume_ml_O2 > maxvolumeoxygen) {
    maxvolumeoxygen = Volume_ml_O2;
  }
}

void FLOW_SENSOR_updateVolumeO2init(float flow_O2) { //flow = liter/min
  totalFlow_O2 += flow_O2;
  Volume_l_O2 = totalFlow_O2 / 60000;
  Volume_ml_O2 = (int)(Volume_l_O2 * 1000);
}

int FLOW_SENSOR_getTotalVolumeIntO2() { // Volume = ml
  return Volume_ml_O2;
}

bool FLOW_SENSOR_getVolumeO2(float *value) {
  if (IS_FLOW_SENSOR_O2_INITIALIZED) {
    *value = Volume_ml_O2;
    return true;
  }
  return false;
}

int FLOW_SENSOR_getMaxVolumeO2() {
  return maxvolumeoxygen;
}

//----------------------------------------------------------------------------------------------------------------
// OXYGEN PID
//----------------------------------------------------------------------------------------------------------------



void FLOW_SENSOR_setK_O2(float k_O2) {
  K_O2 = k_O2;
}


unsigned long FLOW_SENSOR_getTime(float fio2) {

  // don't add oxygen below 25% requested
  if (fio2 < fio2min) {
    valveControlIterations = 0; //reset to zero
    fio2 = 0.2;

    return 0;
  }

  // prevent inflation of bag
  if (fio2 > fio2max) {
    fio2 = fio2max;
  }


  //setting on GUI has changed to higher value
  if (fio2 > previous_target_fio2 ) {
   
    valveControlIterations = 0; //reset to zero when target value is changed
    previous_target_fio2 = fio2;

    //select a preset value as a start approximation
    for (int i = 10; i > 0; i--) {
      if (target_fio2 * 10 < i) {
        valvetime = valvetime_presetvalues[i - 1]; // select presetvalue
        valvetime = maxvolumepatient * valvetime / 800; // correction for tidal volume
      }
    }


  }




  //setting on GUI has changed to lower value
  if (fio2 < previous_target_fio2) {
  
    valveControlIterations = 0; //store the fact that we are starting a correction
    previous_target_fio2 = fio2;

    //select a preset value as a start approximation
    for (int i = 10; i > 2; i--) {
      if (target_fio2 * 10 < i) {
        valvetime = valvetime_presetvalues[i - 2]; // select presetvalue
        valvetime = maxvolumepatient * valvetime / 800; // correction for tidal volume
      }
    }

  }

  valveControlIterations += 1;

  if (valveControlIterations >= startupiterations) {
    //this is Thomas' controller taking over
    wantedoxygenvolume = maxvolumepatient * (fio2 - o2air) / (o2oxy - o2air);
    // calulate corresponding time to open valve
   // K_O2 = wantedoxygenvolume / valvetime; //change K_O2 for after initial iterations

    // calculate wanted oxygen volume
    wantedoxygenvolume = maxvolumepatient * (fio2 - o2air) / (o2oxy - o2air);
    // calulate corresponding time to open valve
    valvetime = K_O2 * wantedoxygenvolume;
  }
  // don't return negative valve time
  if (valvetime < 0) {
    valvetime = 0;
  }

  return valvetime;
}


/* --ORIGINAL CODE FROM THOMAS --
  unsigned long FLOW_SENSOR_getTime(float fio2){
  // don't add oxygen below 25% requested
  if (fio2 < fio2min){
    return 0;
  }
  // prevent inflation of bag
  if(fio2 > fio2max){
    fio2 = fio2max;
  }
  // calculate wanted oxygen volume
  wantedoxygenvolume = maxvolumepatient * (fio2-o2air)/(o2oxy-o2air);
  // calulate corresponding time to open valve
  valvetime = K_O2 * wantedoxygenvolume;

  // don't return negative valve time
  if (valvetime < 0){
    valvetime = 0;
  }
  return valvetime;
  }

*/


void FLOW_SENSOR_updateK_O2() {
if (valveControlIterations<startupiterations){
//this resets controller to initial values when targetfio2 has been changed

  Vo2_cum_error=0;
  K_O2=1;
}


  // calculate running average of supplied oxygen volume
  totalO2 = totalO2 - readingsO2[readIndexO2];
  readingsO2[readIndexO2] = maxvolumeoxygen;
  totalO2 = totalO2 + readingsO2[readIndexO2];
  readIndexO2 = readIndexO2 + 1;

  if (readIndexO2 >= numReadingsO2) readIndexO2 = 0;
  maxvolumeoxygenaveraged = totalO2 / numReadingsO2;

  // calculate error and update K_O2
  if (wantedoxygenvolume == 0) wantedoxygenvolume = 1;
  Vo2_cum_error += Vo2_error;

  Vo2_error = (wantedoxygenvolume - maxvolumeoxygenaveraged) / wantedoxygenvolume;
   if (valveControlIterations >= startupiterations) {
  float K_O2_new = K_O2 + Cp_O2 * Vo2_error + Ci_O2 * Vo2_cum_error;
  // check if we are not delivering too much oxygen

  if ((maxvolumeoxygenaveraged > 1.25 * wantedoxygenvolume) && (K_O2_new > K_O2)) {
    // don't increase K_02
  }
  else {
    if (valveControlIterations >= startupiterations) {
      K_O2 = K_O2_new;
    }
  }
  if (K_O2 < 0) {
    K_O2 = 0;
  }

   }
  //  DEBUGserialprint("error: ");
  //  DEBUGserialprintln(Vo2_error);
  //  DEBUGserialprint("cumulative error: ");
  //  DEBUGserialprintln(Vo2_cum_error*wantedoxygenvolume);
  //  DEBUGserialprint("K: ");
  //  DEBUGserialprintln(K_O2);
  //  DEBUGserialprint("valveTime: ");
  //  DEBUGserialprintln(valvetime);
  //  DEBUGserialprint("Vpatient: ");
  //  DEBUGserialprintln(maxvolumepatient);
  //  DEBUGserialprint("Voxygenwanted: ");
  //  DEBUGserialprintln(wantedoxygenvolume);
  //  DEBUGserialprint("V02: ");
  //  DEBUGserialprintln(maxvolumeoxygen);
  //  DEBUGserialprint("FIO2: ");
  //  DEBUGserialprintln(FLOW_SENSOR_getFIO2());
}

float FLOW_SENSOR_getFIO2() {
  if (maxvolumepatient == 0) maxvolumepatient = 10; // avoid divide by zero
  float fio2measured = ((o2oxy - o2air) * maxvolumeoxygenaveraged / maxvolumepatient) + o2air;

  if (fio2measured > 1) {
    fio2measured = 1;
  }
  if (fio2measured < 0) {
    fio2measured = 0;
  }
  return fio2measured;
}
float FLOW_SENSOR_getFIO2instant() {
  //added this function to send non-averaged fio2measured to GUI, without affecting the controller
  if (maxvolumepatient == 0) maxvolumepatient = 10; // avoid divide by zero
  float fio2measured = ((o2oxy - o2air) * maxvolumeoxygen / maxvolumepatient) + o2air;

  if (fio2measured > 1) {
    fio2measured = 1;
  }
  if (fio2measured < 0) {
    fio2measured = 0;
  }
  return fio2measured;
}

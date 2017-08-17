// // SHUTTER v29 and before
// v29 and before we assigned these params at compile
// // CCW = open shutter
// // const double CCWSHUTTERCURRENTTHRESHOLD = 50;
// // const unsigned CCWSHUTTERTIMEOUT = 1400;
// // const unsigned CCWSTOPMILLISECS = 200;
// // CW = close
// // const double CWSHUTTERCURRENTTHRESHOLD = 50;
// // const unsigned CWSHUTTERTIMEOUT = 2000;
// // const unsigned CWSTOPMILLISECS = 200;
//
// SHUTTER v30+ new definitions same parameters in eeprom
// shutter open/close parameters
// const int default_open_params[3] = {50,2000,200};   //milliamps,millisecs,millisecs 
// const int default_close_params[3]={50,1400,200};
// equivalency
// ee.open_params[0] -- CCWSHUTTERCURRENTTHRESHOLD
// ee.open_params[1] -- CCWSHUTTERTIMEOUT
// ee.open_params[2] -- CCWSTOPMILLISECS
// ee.close_params[0] -- CWSHUTTERCURRENTTHRESHOLD
// ee.close_params[1] -- CWSHUTTERTIMEOUT
// ee.close_params[2] -- CWSTOPMILLISECS

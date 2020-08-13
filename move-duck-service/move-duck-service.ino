/*
 * Arrowhead meArm 'move-duck' service demo by Balazs Riskutia <balazs.riskutia@outlook.hu>
 * Based on the Arrowhead Adapter - refer to ArrowheadESP library and ...
 * Implements a service that picks up a duck at point A and puts it on point B.
 */
 
#include <NTPClient.h>
#include <ArduinoJson.h>
#include <ArrowheadESP.h>
#include "meArm.h"
#include <Servo.h>

// Server and service parameters
// TODO: change params, if needed!
#define SERVER_PORT 8080 
#define SERVICE_URI "/move-duck"

// position struct definition
struct Position {
  int x;
  int y;
  int z;
};

ArrowheadESP Arrowhead;

Servo gripper;
meArm arm;

// pre-defined positions
Position reference = { 0, 200, 0 }; // this position is marked by the QR code
Position standby = { 0, 120, 80 }; // standby position of the robot
int restingHeight = 50; // this is the assumed z coordinate of a resting duck
int carryingHeight = 140; // this will be the z coordinate at which the duck will be moved between points
int rMax = 200; // the radius of the half circle inside which the robot is enabled to operate

// This function will handle the incoming service request
void handleServiceRequest() {

    StaticJsonDocument<500> root;
    String response;

    // check if the provided coordinates are valid in format
    if(Arrowhead.getWebServer().arg("ax") == ""
      || Arrowhead.getWebServer().arg("ay") == ""
      || Arrowhead.getWebServer().arg("bx") == ""
      || Arrowhead.getWebServer().arg("by") == "") {
        root["status"] = "error";
        root["message"] = "Missing coordinates.";
        serializeJson(root, response);
        Arrowhead.getWebServer().send(200, "application/json", response); // return with error response
        return;
     }

    // calculate points inside the arm's coordinate-system
    Position a = { Arrowhead.getWebServer().arg("ax").toInt() + reference.x, Arrowhead.getWebServer().arg("ay").toInt() + reference.y, 0 };
    Position b = { Arrowhead.getWebServer().arg("bx").toInt() + reference.x, Arrowhead.getWebServer().arg("by").toInt() + reference.y, 0 };

    bool hasError = false;
    String errorMessage = "";
    double aRadius = sqrt(pow(a.x, 2) + pow(a.y, 2));
    double bRadius = sqrt(pow(b.x, 2) + pow(b.y, 2));

    // check whether the provided points are inside the half circle
    if(a.y < 0 || aRadius > rMax) {
      errorMessage = "Starting point is out of reach";
      hasError = true;
    }
    if(b.y < 0 || bRadius > rMax) {
      errorMessage = "Destination point is out of reach";
      hasError = true;
    }

    // return with error
    if(hasError) {
      root["status"] = "error";
      root["message"] = errorMessage;
      serializeJson(root, response);
      Arrowhead.getWebServer().send(200, "application/json", response);
      return;
    }

    // move the robot arm accordingly
    arm.gotoPoint(standby.x,standby.y,carryingHeight);
    openGripper();
    delay(1000);
    arm.gotoPoint(a.x,a.y,carryingHeight);
    delay(1000);
    arm.gotoPoint(a.x,a.y,restingHeight);
    delay(600);
    closeGripper();
    delay(1000);
    arm.gotoPoint(a.x,a.y,carryingHeight);
    delay(1000);
    arm.gotoPoint(b.x,b.y,carryingHeight);
    delay(1000);
    arm.gotoPoint(b.x,b.y,restingHeight);
    delay(600);
    openGripper();
    delay(1000);
    arm.gotoPoint(b.x,b.y,carryingHeight);
    delay(1000);
    arm.gotoPoint(standby.x,standby.y,standby.z);
    delay(300);

    // return with success
    root["status"] = "success";
    serializeJson(root, response);
    Arrowhead.getWebServer().send(200, "application/json", response);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Arrowhead.getArrowheadESPFS().loadConfigFile("netConfig.json"); // loads network config from file system
  Arrowhead.getArrowheadESPFS().loadSSLConfigFile("sslConfig.json"); // loads ssl config from file system
  Arrowhead.getArrowheadESPFS().loadProviderConfigFile("providerConfig.json"); // loads provider config from file system
  
  // Set the Address and port of the Service Registry.
  Arrowhead.setServiceRegistryAddress(
    Arrowhead.getArrowheadESPFS().getProviderInfo().serviceRegistryAddress,
    Arrowhead.getArrowheadESPFS().getProviderInfo().serviceRegistryPort
  );

  bool startupSuccess = Arrowhead.begin(); // true of connection to WiFi and loading Certificates is successful
  if(startupSuccess){

    // Check if service registry
    String response = "";
    int statusCode = Arrowhead.serviceRegistryEcho(&response);
    Serial.print("Status code from server: ");
    Serial.println(statusCode);
    Serial.print("Response body from server: ");
    Serial.println(response);

    String serviceRegistryEntry = "{\"endOfValidity\":\"2021-12-05 12:00:00\",\"interfaces\":[\"HTTP-INSECURE-SenML\"],\"providerSystem\":{\"address\":\" "+ Arrowhead.getIP() +"\",\"authenticationInfo\":\""+ Arrowhead.getArrowheadESPFS().getSSLInfo().publicKey +"\",\"port\":"+ SERVER_PORT +",\"systemName\":\""+ Arrowhead.getArrowheadESPFS().getProviderInfo().systemName +"\"},\"secure\":\"CERTIFICATE\",\"serviceDefinition\":\"move-duck\",\"serviceUri\":\""+SERVICE_URI+"\",\"version\":1}";  

    response = "";
    statusCode = Arrowhead.serviceRegistryRegister(serviceRegistryEntry.c_str(), &response);
    Serial.print("Status code from server: ");
    Serial.println(statusCode);
    Serial.print("Response body from server: ");
    Serial.println(response);
  }

  Arrowhead.getWebServer().on(SERVICE_URI, handleServiceRequest);
  Arrowhead.getWebServer().begin(SERVER_PORT);
  
  arm.begin(13, 16, 14, 12); // set PINs (base, shoulder, elbow, gripper)
  gripper.attach(12); // set gripper PIN

  arm.gotoPoint(standby.x,standby.y,standby.z); // go to standby position
  openGripper();
} 

// define custom grip values
void openGripper() {
  gripper.write(80); 
}

// define custom grip values
void closeGripper() {
  gripper.write(130);
}

void loop() {
  Arrowhead.loop(); // keep network connection up
  yield();
}

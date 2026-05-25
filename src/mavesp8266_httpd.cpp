/****************************************************************************
 *
 * Copyright (c) 2015, 2016 Gus Grubba. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
        0,
        0,
        0,
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavesp8266_httpd.cpp
 * ESP8266 Wifi AP, MavLink UART/UDP Bridge
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */

#include <ESP8266WebServer.h>

#include "mavesp8266.h"
#include "mavesp8266_httpd.h"
#include "mavesp8266_parameters.h"
#include "mavesp8266_gcs.h"
#include "mavesp8266_vehicle.h"

const char PROGMEM kTEXTPLAIN[]  = "text/plain";
const char PROGMEM kTEXTHTML[]   = "text/html";
const char PROGMEM kACCESSCTL[]  = "Access-Control-Allow-Origin";
const char PROGMEM kUPLOADFORM[] = "<h1><a href='/'>MAVLink WiFi Bridge</a></h1><form method='POST' action='/upload' enctype='multipart/form-data'><input type='file' name='update'><br><input type='submit' value='Update'></form>";
const char PROGMEM kHEADER[]     = "<!doctype html><html><head><title>MavLink Bridge</title></head><body><h1><a href='/'>MAVLink WiFi Bridge</a></h1>";
const char PROGMEM kBADARG[]     = "BAD ARGS";
const char PROGMEM kAPPJSON[]    = "application/json";
const char PROGMEM kTELEMETRYPAGE[] = R"rawliteral(
<!doctype html>
<html>
<head>
<title>MavLink Bridge Telemetry</title>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<style>
body{font-family:Arial,sans-serif;margin:0;padding:0;background:#fafafa;color:#111;}
h1{margin:10px 12px;font-size:24px;}
h1 a{text-decoration:none;color:#111;}
#status{background:#d4edda;border:1px solid #28a745;color:#155724;font-weight:bold;padding:10px;margin:10px 12px;border-radius:5px;font-size:clamp(15px,3.7vw,26px);text-align:center;}
#vals{display:grid;grid-template-columns:repeat(4,minmax(120px,1fr));gap:8px;margin:10px 12px;}
.telem-item{border:1px solid #ccc;padding:8px 6px;background:#fff;border-radius:6px;max-width:180px;justify-self:center;text-align:center;}
.telem-label{font-weight:600;display:block;margin-bottom:6px;font-size:clamp(11px,1.2vw,13px);letter-spacing:0.04em;text-transform:uppercase;opacity:0.8;}
.telem-value{font-size:clamp(27px,4.0vw,48px);font-weight:700;line-height:1.1;}
@media (max-width:1100px){#vals{grid-template-columns:repeat(3,minmax(120px,1fr));}}
@media (max-width:820px){#vals{grid-template-columns:repeat(2,minmax(120px,1fr));}}
@media (max-width:560px){
    h1{margin:8px 10px;font-size:21px;}
    #status{margin:8px 10px;padding:8px;font-size:clamp(14px,4.1vw,20px);}
    #vals{margin:8px 8px;gap:5px;grid-template-columns:repeat(3,minmax(0,1fr));}
    .telem-item{padding:7px 4px;max-width:100%;justify-self:stretch;}
    .telem-label{margin-bottom:4px;font-size:11px;}
    .telem-value{font-size:clamp(20px,7.4vw,32px);}
}
@media (max-width:340px){#vals{grid-template-columns:repeat(2,minmax(0,1fr));}}
</style>
</head>
<body>
<h1><a href='/'>MAVLink WiFi Bridge</a></h1>
<div id='status'>Status: waiting for data...</div>
<div id='vals'>waiting for data</div>
<script>
(function(){
var inFlight=false;
var reqId=0;
function oneDec(v){var n=Number(v);return isNaN(n)?'--.-':n.toFixed(1);}
function relDec(v,p,h){var angle=Number(v);var pitch=h?Number(p):0;var out=angle-pitch;return isNaN(out)?'--.-':out.toFixed(1);}
function setStatus(text,isBad){
    var e=document.getElementById('status');
    e.textContent='Status: '+text;
    if(isBad){
        e.style.background='#f8d7da';
        e.style.borderColor='#dc3545';
        e.style.color='#721c24';
    }else{
        e.style.background='#d4edda';
        e.style.borderColor='#28a745';
        e.style.color='#155724';
    }
}
function updateFoils(d){
    var arm='NO HEARTBEAT';
    if(d.hb_seen){arm=d.armed?'ARMED':'DISARMED';}
    setStatus(arm,false);
    var html='';
    html+='<div class="telem-item"><span class="telem-label">MAIN BB</span><span class="telem-value">'+relDec(d.main_ps,d.pitch,d.att_seen)+' &deg;</span></div>';
    html+='<div class="telem-item"><span class="telem-label">PITCH</span><span class="telem-value">'+(d.att_seen?oneDec(d.pitch)+' &deg;':'--.- &deg;')+'</span></div>';
    html+='<div class="telem-item"><span class="telem-label">MAIN TB</span><span class="telem-value">'+relDec(d.main_sb,d.pitch,d.att_seen)+' &deg;</span></div>';
    html+='<div class="telem-item"><span class="telem-label">RUDDER BB</span><span class="telem-value">'+relDec(d.rudder_ps,d.pitch,d.att_seen)+' &deg;</span></div>';
    html+='<div class="telem-item"><span class="telem-label">ROLL</span><span class="telem-value">'+(d.att_seen?oneDec(d.roll)+' &deg;':'--.- &deg;')+'</span></div>';
    html+='<div class="telem-item"><span class="telem-label">RUDDER TB</span><span class="telem-value">'+relDec(d.rudder_sb,d.pitch,d.att_seen)+' &deg;</span></div>';
    html+='<div class="telem-item"><span class="telem-label">HEAVE</span><span class="telem-value">'+oneDec(d.heave)+' m</span></div>';    
    document.getElementById('vals').innerHTML=html;
}
function poll(){
    if(inFlight){return;}
    inFlight=true;
    var x=new XMLHttpRequest();
    var thisReq=++reqId;
    x.timeout=1200;
    x.onreadystatechange=function(){
        if(x.readyState!==4){return;}
        if(thisReq!==reqId){return;}
        inFlight=false;
        if(x.status===200){
            try {
                updateFoils(JSON.parse(x.responseText));
            } catch(e) {
                setStatus('bad telemetry payload',true);
            }
        } else {
            setStatus('telemetry link slow/unavailable',true);
        }
    };
    x.ontimeout=function(){if(thisReq===reqId){inFlight=false;setStatus('telemetry timeout',true);}};
    x.onerror=function(){if(thisReq===reqId){inFlight=false;setStatus('telemetry request error',true);}};
    x.open('GET','/api/foils?t=' + Date.now(),true);
    x.send();
}
poll();
setInterval(poll,1000);
})();
</script>
</body>
</html>
)rawliteral";

const char* kBAUD       = "baud";
const char* kPWD        = "pwd";
const char* kSSID       = "ssid";
const char* kPWDSTA     = "pwdsta";
const char* kSSIDSTA    = "ssidsta";
const char* kIPSTA      = "ipsta";
const char* kGATESTA    = "gatewaysta";
const char* kSUBSTA     = "subnetsta";
const char* kCPORT      = "cport";
const char* kHPORT      = "hport";
const char* kCHANNEL    = "channel";
const char* kDEBUG      = "debug";
const char* kREBOOT     = "reboot";
const char* kPOSITION   = "position";
const char* kMODE       = "mode";

const char* kFlashMaps[7] = {
    "512KB (256/256)",
    "256KB",
    "1MB (512/512)",
    "2MB (512/512)",
    "4MB (512/512)",
    "2MB (1024/1024)",
    "4MB (1024/1024)"
};

static uint32_t flash = 0;
static char paramCRC[12] = {""};

ESP8266WebServer    webServer(80);
MavESP8266Update*   updateCB    = NULL;
bool                started     = false;

//---------------------------------------------------------------------------------
void setNoCacheHeaders() {
    webServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    webServer.sendHeader("Pragma", "no-cache");
    webServer.sendHeader("Expires", "0");
}

//---------------------------------------------------------------------------------
void returnFail(String msg) {
    webServer.send(500, FPSTR(kTEXTPLAIN), msg + "\r\n");
}

//---------------------------------------------------------------------------------
void respondOK() {
    webServer.send(200, FPSTR(kTEXTPLAIN), "OK");
}

//---------------------------------------------------------------------------------
void handle_update() {
    webServer.sendHeader("Connection", "close");
    webServer.sendHeader(FPSTR(kACCESSCTL), "*");
    webServer.send(200, FPSTR(kTEXTHTML), FPSTR(kUPLOADFORM));
}

//---------------------------------------------------------------------------------
void handle_upload() {
    webServer.sendHeader("Connection", "close");
    webServer.sendHeader(FPSTR(kACCESSCTL), "*");
    webServer.send(200, FPSTR(kTEXTPLAIN), (Update.hasError()) ? "FAIL" : "OK");
    if(updateCB) {
        updateCB->updateCompleted();
    }
    ESP.restart();
}

//---------------------------------------------------------------------------------
void handle_upload_status() {
    bool success  = true;
    if(!started) {
        started = true;
        if(updateCB) {
            updateCB->updateStarted();
        }
    }
    HTTPUpload& upload = webServer.upload();
    if(upload.status == UPLOAD_FILE_START) {
        #ifdef DEBUG_SERIAL
            DEBUG_SERIAL.setDebugOutput(true);
        #endif
        WiFiUDP::stopAll();
        #ifdef DEBUG_SERIAL
            DEBUG_SERIAL.printf("Update: %s\n", upload.filename.c_str());
        #endif
        uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
        if(!Update.begin(maxSketchSpace)) {
            #ifdef DEBUG_SERIAL
                Update.printError(DEBUG_SERIAL);
            #endif
            success = false;
        }
    } else if(upload.status == UPLOAD_FILE_WRITE) {
        if(Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            #ifdef DEBUG_SERIAL
                Update.printError(DEBUG_SERIAL);
            #endif
            success = false;
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
            #ifdef DEBUG_SERIAL
                DEBUG_SERIAL.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
            #endif
        } else {
            #ifdef DEBUG_SERIAL
                Update.printError(DEBUG_SERIAL);
            #endif
            success = false;
        }
        #ifdef DEBUG_SERIAL
            DEBUG_SERIAL.setDebugOutput(false);
        #endif
    }
    yield();
    if(!success) {
        if(updateCB) {
            updateCB->updateError();
        }
    }
}

//---------------------------------------------------------------------------------
void handle_getParameters()
{
    String message = FPSTR(kHEADER);
    message += "<p>Parameters</p><table><tr><td width=\"240\">Name</td><td>Value</td></tr>";
    for(int i = 0; i < MavESP8266Parameters::ID_COUNT; i++) {
		if(i == getWorld()->getParameters()->ID_FWVER)
		{
			message += "<tr><td>";
            message += getWorld()->getParameters()->getAt(i)->id;
            message += "</td>";
            message += "<td>";
            message += MAVESP8266_VERSION_MAJOR;
            message += ".";
            message += MAVESP8266_VERSION_MINOR;
            message += ".";
            message += MAVESP8266_VERSION_BUILD;
            message += "</td></tr>";
		}
        else if(i == getWorld()->getParameters()->ID_MODE)
        {
            message += "<tr><td>";
            message += getWorld()->getParameters()->getAt(i)->id;
            message += "</td>";
            message += "<td>";
            if(getWorld()->getParameters()->getWifiMode() == WIFI_MODE_AP)
            {
                message += "AP";
            }
            else
            {
                message += "STA";
            }
            message += "</td></tr>";
        }
        else if(i == getWorld()->getParameters()->ID_IPADDRESS)
        {
            message += "<tr><td>";
            message += getWorld()->getParameters()->getAt(i)->id;
            message += "</td>";
            message += "<td>";
            message += getWorld()->getParameters()->getLocalIPAddressInString();
            message += "</td></tr>";
        }
        else if(i == getWorld()->getParameters()->ID_SSID1)
        {
            message += "<tr><td>";
            message += getWorld()->getParameters()->getAt(i)->id;
            message += "</td>";
            message += "<td>";
            message += getWorld()->getParameters()->getWifiSsid();
            message += "</td></tr>";
        }
        else if(i > getWorld()->getParameters()->ID_SSID1 && i <= getWorld()->getParameters()->ID_SSID4) {}
        else if(i == getWorld()->getParameters()->ID_PASS1)
        {
            message += "<tr><td>";
            message += getWorld()->getParameters()->getAt(i)->id;
            message += "</td>";
            message += "<td>";
            message += getWorld()->getParameters()->getWifiPassword();
            message += "</td></tr>";
        }
        else if(i > getWorld()->getParameters()->ID_PASS1 && i <= getWorld()->getParameters()->ID_PASS4) {}
        else if(i == getWorld()->getParameters()->ID_SSIDSTA1)
        {
            message += "<tr><td>";
            message += getWorld()->getParameters()->getAt(i)->id;
            message += "</td>";
            message += "<td>";
            message += getWorld()->getParameters()->getWifiStaSsid();
            message += "</td></tr>";
        }
        else if(i > getWorld()->getParameters()->ID_SSIDSTA1 && i <= getWorld()->getParameters()->ID_SSIDSTA4) {}
        else if(i == getWorld()->getParameters()->ID_PASSSTA1)
        {
            message += "<tr><td>";
            message += getWorld()->getParameters()->getAt(i)->id;
            message += "</td>";
            message += "<td>";
            message += getWorld()->getParameters()->getWifiStaPassword();
            message += "</td></tr>";
        }
        else if(i > getWorld()->getParameters()->ID_PASSSTA1 && i <= getWorld()->getParameters()->ID_PASSSTA4) {}
        else // integer values
        {
            message += "<tr><td>";
            message += getWorld()->getParameters()->getAt(i)->id;
            message += "</td>";
            unsigned long val = 0;
            if(getWorld()->getParameters()->getAt(i)->type == MAV_PARAM_TYPE_UINT32)
                val = (unsigned long)*((uint32_t*)getWorld()->getParameters()->getAt(i)->value);
            else if(getWorld()->getParameters()->getAt(i)->type == MAV_PARAM_TYPE_UINT16)
                val = (unsigned long)*((uint16_t*)getWorld()->getParameters()->getAt(i)->value);
            else
                val = (unsigned long)*((int8_t*)getWorld()->getParameters()->getAt(i)->value);

            message += "<td>";
            message += val;
            message += "</td></tr>";
        }
    }
    message += "</table>";
    message += "</body>";
    webServer.send(200, FPSTR(kTEXTHTML), message);
}

//---------------------------------------------------------------------------------
static void handle_root()
{
    String message = FPSTR(kHEADER);
    message += "Version: ";
    char vstr[30];
    snprintf(vstr, sizeof(vstr), "%u.%u.%u", MAVESP8266_VERSION_MAJOR, MAVESP8266_VERSION_MINOR, MAVESP8266_VERSION_BUILD);
    message += vstr;
    message += "<p>\n";
    message += "<ul>\n";
    message += "<li><a href='/getstatus'>Get Status</a>\n";
    message += "<li><a href='/telemetry'>Telemetry</a>\n";
    message += "<li><a href='/mavlink_analyser'>MAVLink Analyser</a>\n";
    message += "<li><a href='/setup'>Setup</a>\n";
    message += "<li><a href='/getparameters'>Get Parameters</a>\n";
    message += "<li><a href='/update'>Update Firmware</a>\n";
    message += "<li><a href='/reboot'>Reboot</a>\n";
    message += "</ul></body>";
    setNoCacheHeaders();
    webServer.send(200, FPSTR(kTEXTHTML), message);
}

//---------------------------------------------------------------------------------
static void handle_setup()
{
    String message = FPSTR(kHEADER);
    message += "<h1>Setup</h1>\n";
    message += "<form action='/setparameters' method='post'>\n";

    message += "WiFi Mode:&nbsp;";
    message += "<input type='radio' name='mode' value='0'";
    if (getWorld()->getParameters()->getWifiMode() == WIFI_MODE_AP) {
        message += " checked";
    }
    message += ">AccessPoint\n";
    message += "<input type='radio' name='mode' value='1'";
    if (getWorld()->getParameters()->getWifiMode() == WIFI_MODE_STA) {
        message += " checked";
    }
    message += ">Station<br>\n";
    
    message += "AP SSID:&nbsp;";
    message += "<input type='text' name='ssid' value='";
    message += getWorld()->getParameters()->getWifiSsid();
    message += "'><br>";

    message += "AP Password (min len 8):&nbsp;";
    message += "<input type='text' name='pwd' value='";
    message += getWorld()->getParameters()->getWifiPassword();
    message += "'><br>";

    message += "WiFi Channel:&nbsp;";
    message += "<input type='text' name='channel' value='";
    message += getWorld()->getParameters()->getWifiChannel();
    message += "'><br>";

    message += "Station SSID:&nbsp;";
    message += "<input type='text' name='ssidsta' value='";
    message += getWorld()->getParameters()->getWifiStaSsid();
    message += "'><br>";

    message += "Station Password:&nbsp;";
    message += "<input type='text' name='pwdsta' value='";
    message += getWorld()->getParameters()->getWifiStaPassword();
    message += "'><br>";

    IPAddress IP;    
    message += "Station IP:&nbsp;";
    message += "<input type='text' name='ipsta' value='";
    IP = getWorld()->getParameters()->getWifiStaIP();
    message += IP.toString();
    message += "'><br>";

    message += "Station Gateway:&nbsp;";
    message += "<input type='text' name='gatewaysta' value='";
    IP = getWorld()->getParameters()->getWifiStaGateway();
    message += IP.toString();
    message += "'><br>";

    message += "Station Subnet:&nbsp;";
    message += "<input type='text' name='subnetsta' value='";
    IP = getWorld()->getParameters()->getWifiStaSubnet();
    message += IP.toString();
    message += "'><br>";

    message += "Host Port:&nbsp;";
    message += "<input type='text' name='hport' value='";
    message += getWorld()->getParameters()->getWifiUdpHport();
    message += "'><br>";

    message += "Client Port:&nbsp;";
    message += "<input type='text' name='cport' value='";
    message += getWorld()->getParameters()->getWifiUdpCport();
    message += "'><br>";
    
    message += "Baudrate:&nbsp;";
    message += "<input type='text' name='baud' value='";
    message += getWorld()->getParameters()->getUartBaudRate();
    message += "'><br>";
    
    message += "<input type='submit' value='Save'>";
    message += "</form>";
    setNoCacheHeaders();
    webServer.send(200, FPSTR(kTEXTHTML), message);
}


//---------------------------------------------------------------------------------
static void handle_getStatus()
{
    if(!flash)
        flash = ESP.getFreeSketchSpace();
    if(!paramCRC[0]) {
        snprintf(paramCRC, sizeof(paramCRC), "%08X", getWorld()->getParameters()->paramHashCheck());
    }
    linkStatus* gcsStatus = getWorld()->getGCS()->getStatus();
    linkStatus* vehicleStatus = getWorld()->getVehicle()->getStatus();
    String message = FPSTR(kHEADER);
    message += "<p>Comm Status</p><table><tr><td width=\"240\">Packets Received from GCS</td><td>";
    message += gcsStatus->packets_received;
    message += "</td></tr><tr><td>Packets Sent to GCS</td><td>";
    message += gcsStatus->packets_sent;
    message += "</td></tr><tr><td>GCS Packets Lost</td><td>";
    message += gcsStatus->packets_lost;
    message += "</td></tr><tr><td>Packets Received from Vehicle</td><td>";
    message += vehicleStatus->packets_received;
    message += "</td></tr><tr><td>Packets Sent to Vehicle</td><td>";
    message += vehicleStatus->packets_sent;
    message += "</td></tr><tr><td>Vehicle Packets Lost</td><td>";
    message += vehicleStatus->packets_lost;
    message += "</td></tr><tr><td>Radio Messages</td><td>";
    message += gcsStatus->radio_status_sent;
    message += "</td></tr></table>";
    message += "<p>System Status</p><table>\n";
    message += "<tr><td width=\"240\">Flash Size</td><td>";
    message += ESP.getFlashChipRealSize();
    message += "</td></tr>\n";
    message += "<tr><td width=\"240\">Flash Available</td><td>";
    message += flash;
    message += "</td></tr>\n";
    message += "<tr><td>RAM Left</td><td>";
    message += String(ESP.getFreeHeap());
    message += "</td></tr>\n";
    message += "<tr><td>Parameters CRC</td><td>";
    message += paramCRC;
    message += "</td></tr>\n";
    message += "</table>";
    message += "</body>";
    setNoCacheHeaders();
    webServer.send(200, FPSTR(kTEXTHTML), message);
}

//---------------------------------------------------------------------------------
void handle_getJLog()
{
    uint32_t position = 0, len;
    if(webServer.hasArg(kPOSITION)) {
        position = webServer.arg(kPOSITION).toInt();
    }
    String logText = getWorld()->getLogger()->getLog(&position, &len);
    char jStart[128];
    snprintf(jStart, 128, "{\"len\":%d, \"start\":%d, \"text\": \"", len, position);
    String payLoad = jStart;
    payLoad += logText;
    payLoad += "\"}";
    webServer.send(200, FPSTR(kAPPJSON), payLoad);
}

//---------------------------------------------------------------------------------
void handle_getFoils()
{
    MavESP8266Vehicle* vehicle = getWorld()->getVehicle();
    const mavlink_m2_state_t* m2 = vehicle->m2State();
    const mavlink_boat_attitude_t* att = vehicle->boatAttitude();
    const mavlink_boat_heave_t* heave = vehicle->boatHeave();
    const mavlink_heartbeat_t* hb = vehicle->getHeartbeat();

    const uint32_t kFoilsHeartbeatStaleMs = HEARTBEAT_TIMEOUT;
    const uint32_t kFoilsAttitudeStaleMs = 3000;
    const uint32_t kFoilsHeaveStaleMs = 3000;
    const uint32_t kFoilsM2StaleMs = 3000;

    bool hbSeen = vehicle->heardFrom() && vehicle->heartbeatSeen() && (vehicle->heartbeatAgeMs() <= kFoilsHeartbeatStaleMs);
    bool attSeen = vehicle->boatAttitudeSeen() && (vehicle->boatAttitudeAgeMs() <= kFoilsAttitudeStaleMs);
    bool heaveSeen = vehicle->boatHeaveSeen() && (vehicle->boatHeaveAgeMs() <= kFoilsHeaveStaleMs);
    bool m2Seen = vehicle->m2StateSeen() && (vehicle->m2StateAgeMs() <= kFoilsM2StaleMs);
    bool armed = hbSeen && ((hb->base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0);

    printf("[FOILS] hbSeen=%d (heardFrom=%d, hbSeen_flag=%d, hb_age=%u, timeout=%u) m2Seen=%d (age=%u) heaveSeen=%d (age=%u) attSeen=%d (age=%u)\n",
        hbSeen, vehicle->heardFrom(), vehicle->heartbeatSeen(), (unsigned int)vehicle->heartbeatAgeMs(), (unsigned int)kFoilsHeartbeatStaleMs,
        m2Seen, (unsigned int)vehicle->m2StateAgeMs(),
        heaveSeen, (unsigned int)vehicle->boatHeaveAgeMs(),
        attSeen, (unsigned int)vehicle->boatAttitudeAgeMs());

    char mainSb[16];
    char mainPs[16];
    char rudderSb[16];
    char rudderPs[16];
    char heaveValue[16];
    char pitch[16];
    char roll[16];

    if(m2Seen) {
        snprintf(mainSb, sizeof(mainSb), "%.1f", m2->main_sb);
        snprintf(mainPs, sizeof(mainPs), "%.1f", m2->main_ps);
        snprintf(rudderSb, sizeof(rudderSb), "%.1f", m2->rudder_sb);
        snprintf(rudderPs, sizeof(rudderPs), "%.1f", m2->rudder_ps);
    } else {
        snprintf(mainSb, sizeof(mainSb), "\"NaN\"");
        snprintf(mainPs, sizeof(mainPs), "\"NaN\"");
        snprintf(rudderSb, sizeof(rudderSb), "\"NaN\"");
        snprintf(rudderPs, sizeof(rudderPs), "\"NaN\"");
    }

    if(heaveSeen) {
        snprintf(heaveValue, sizeof(heaveValue), "%.1f", heave->heave);
    } else {
        snprintf(heaveValue, sizeof(heaveValue), "\"NaN\"");
    }

    if(attSeen) {
        snprintf(pitch, sizeof(pitch), "%.1f", att->pitch);
        snprintf(roll, sizeof(roll), "%.1f", att->roll);
    } else {
        snprintf(pitch, sizeof(pitch), "\"NaN\"");
        snprintf(roll, sizeof(roll), "\"NaN\"");
    }

    char message[448];
    snprintf(
        message,
        sizeof(message),
        "{\"m2_seen\":%s,\"att_seen\":%s,\"heave_seen\":%s,\"hb_seen\":%s,\"armed\":%s,\"m2_age_ms\":%u,\"main_sb\":%s,\"main_ps\":%s,\"rudder_sb\":%s,\"rudder_ps\":%s,\"heave\":%s,\"pitch\":%s,\"roll\":%s}",
        m2Seen ? "true" : "false",
        attSeen ? "true" : "false",
        heaveSeen ? "true" : "false",
        hbSeen ? "true" : "false",
        armed ? "true" : "false",
        (unsigned int)vehicle->m2StateAgeMs(),
        mainSb,
        mainPs,
        rudderSb,
        rudderPs,
        heaveValue,
        pitch,
        roll
    );
    setNoCacheHeaders();
    webServer.send(200, FPSTR(kAPPJSON), message);
}

//---------------------------------------------------------------------------------
static void handle_telemetry()
{
    setNoCacheHeaders();
    webServer.send_P(200, kTEXTHTML, kTELEMETRYPAGE);
}

//---------------------------------------------------------------------------------
static void handle_reboot_autopilot()
{
    MavESP8266Vehicle* vehicle = getWorld()->getVehicle();
    if(!vehicle->heardFrom()) {
        returnFail("No vehicle heartbeat");
        return;
    }

    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        vehicle->systemID(),
        MAV_COMP_ID_UDP_BRIDGE,
        &msg,
        vehicle->systemID(),
        vehicle->componentID(),
        MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0,
        1.f,
        1.f,
        1.f,
        1.f,
        1.f,
        1.f,
        1.f
    );
    vehicle->sendMessage(&msg);
    respondOK();
}

//---------------------------------------------------------------------------------
static void handle_mavlink_analyser()
{
    String message = FPSTR(kHEADER);
    message += "<h2>MAVLink Message Analyser</h2>\n";
    message += "<style>\n";
    message += "table{border-collapse:collapse;margin:20px 0;}\n";
    message += "th,td{border:1px solid #ccc;padding:8px 12px;text-align:left;}\n";
    message += "th{background-color:#333;color:#fff;}\n";
    message += "tr:nth-child(even){background-color:#f9f9f9;}\n";
    message += "</style>\n";
    message += "<form method='POST' action='/mavlink_analyser/reboot' onsubmit=\"return confirm('Reboot the autopilot?');\">";
    message += "<input type='submit' value='Reboot Autopilot'>";
    message += "</form>\n";
    message += "<table>\n";
    message += "<tr><th>Message ID</th><th>Count</th></tr>\n";
    
    MavESP8266Vehicle* vehicle = getWorld()->getVehicle();
    uint16_t count = vehicle->messageCounterCount();
    
    for(uint16_t i = 0; i < count; i++) {
        uint32_t msgid = vehicle->messageCounterId(i);
        uint32_t msgcount = vehicle->messageCounterValue(i);
        message += "<tr><td>";
        message += msgid;
        message += "</td><td>";
        message += msgcount;
        message += "</td></tr>\n";
    }
    
    if(vehicle->messageCounterOverflow() > 0) {
        message += "<tr><td colspan='2' style='background-color:#ffcccc;'>";
        message += "Overflow: ";
        message += vehicle->messageCounterOverflow();
        message += " additional message types</td></tr>\n";
    }
    
    message += "</table>\n";
    message += "<p><a href='/'>Back to Home</a></p>\n";
    message += "</body>";
    setNoCacheHeaders();
    webServer.send(200, FPSTR(kTEXTHTML), message);
}

//---------------------------------------------------------------------------------
void handle_getJSysInfo()
{
    if(!flash)
        flash = ESP.getFreeSketchSpace();
    if(!paramCRC[0]) {
        snprintf(paramCRC, sizeof(paramCRC), "%08X", getWorld()->getParameters()->paramHashCheck());
    }
    uint32_t fid = spi_flash_get_id();
    char message[512];
    snprintf(message, 512,
        "{ "
        "\"size\": \"%s\", "
        "\"id\": \"0x%02lX 0x%04lX\", "
        "\"flashfree\": \"%u\", "
        "\"heapfree\": \"%u\", "
        "\"logsize\": \"%u\", "
        "\"paramcrc\": \"%s\""
        " }",
        kFlashMaps[system_get_flash_size_map()],
        (long unsigned int)(fid & 0xff), (long unsigned int)((fid & 0xff00) | ((fid >> 16) & 0xff)),
        flash,
        ESP.getFreeHeap(),
        getWorld()->getLogger()->getPosition(),
        paramCRC
    );
    webServer.send(200, "application/json", message);
}

//---------------------------------------------------------------------------------
void handle_getJSysStatus()
{
    bool reset = false;
    if(webServer.hasArg("r")) {
        reset = webServer.arg("r").toInt() != 0;
    }
    linkStatus* gcsStatus = getWorld()->getGCS()->getStatus();
    linkStatus* vehicleStatus = getWorld()->getVehicle()->getStatus();
    if(reset) {
        memset(gcsStatus,     0, sizeof(linkStatus));
        memset(vehicleStatus, 0, sizeof(linkStatus));
    }
    char message[512];
    snprintf(message, 512,
        "{ "
        "\"gpackets\": \"%u\", "
        "\"gsent\": \"%u\", "
        "\"glost\": \"%u\", "
        "\"vpackets\": \"%u\", "
        "\"vsent\": \"%u\", "
        "\"vlost\": \"%u\", "
        "\"radio\": \"%u\", "
        "\"buffer\": \"%u\""
        " }",
        gcsStatus->packets_received,
        gcsStatus->packets_sent,
        gcsStatus->packets_lost,
        vehicleStatus->packets_received,
        vehicleStatus->packets_sent,
        vehicleStatus->packets_lost,
        gcsStatus->radio_status_sent,
        vehicleStatus->queue_status
    );
    webServer.send(200, "application/json", message);
}

//---------------------------------------------------------------------------------
void handle_setParameters()
{
    if(webServer.args() == 0) {
        returnFail(kBADARG);
        return;
    }
    bool ok = false;
    bool reboot = false;
    if(webServer.hasArg(kBAUD)) {
        ok = true;
        getWorld()->getParameters()->setUartBaudRate(webServer.arg(kBAUD).toInt());
    }
    if(webServer.hasArg(kPWD)) {
        ok = true;
        getWorld()->getParameters()->setWifiPassword(webServer.arg(kPWD).c_str());
    }
    if(webServer.hasArg(kSSID)) {
        ok = true;
        getWorld()->getParameters()->setWifiSsid(webServer.arg(kSSID).c_str());
    }
    if(webServer.hasArg(kPWDSTA)) {
        ok = true;
        getWorld()->getParameters()->setWifiStaPassword(webServer.arg(kPWDSTA).c_str());
    }
    if(webServer.hasArg(kSSIDSTA)) {
        ok = true;
        getWorld()->getParameters()->setWifiStaSsid(webServer.arg(kSSIDSTA).c_str());
    }
    if(webServer.hasArg(kIPSTA)) {
        IPAddress ip;
        ip.fromString(webServer.arg(kIPSTA).c_str());
        getWorld()->getParameters()->setWifiStaIP(ip);
    }
    if(webServer.hasArg(kGATESTA)) {
        IPAddress ip;
        ip.fromString(webServer.arg(kGATESTA).c_str());
        getWorld()->getParameters()->setWifiStaGateway(ip);
    }
    if(webServer.hasArg(kSUBSTA)) {
        IPAddress ip;
        ip.fromString(webServer.arg(kSUBSTA).c_str());
        getWorld()->getParameters()->setWifiStaSubnet(ip);
    }
    if(webServer.hasArg(kCPORT)) {
        ok = true;
        getWorld()->getParameters()->setWifiUdpCport(webServer.arg(kCPORT).toInt());
    }
    if(webServer.hasArg(kHPORT)) {
        ok = true;
        getWorld()->getParameters()->setWifiUdpHport(webServer.arg(kHPORT).toInt());
    }
    if(webServer.hasArg(kCHANNEL)) {
        ok = true;
        getWorld()->getParameters()->setWifiChannel(webServer.arg(kCHANNEL).toInt());
    }
    if(webServer.hasArg(kDEBUG)) {
        ok = true;
        getWorld()->getParameters()->setDebugEnabled(webServer.arg(kDEBUG).toInt());
    }
    if(webServer.hasArg(kMODE)) {
        ok = true;
        getWorld()->getParameters()->setWifiMode(webServer.arg(kMODE).toInt());
    }
    if(webServer.hasArg(kREBOOT)) {
        ok = true;
        reboot = webServer.arg(kREBOOT) == "1";
    }
    if(ok) {
        getWorld()->getParameters()->saveAllToEeprom();
        //-- Send new parameters back
        handle_getParameters();
        if(reboot) {
            delay(100);
            ESP.restart();
        }
    } else
        returnFail(kBADARG);
}

//---------------------------------------------------------------------------------
static void handle_reboot()
{
    String message = FPSTR(kHEADER);
    message += "rebooting ...</body>\n";
    setNoCacheHeaders();
    webServer.send(200, FPSTR(kTEXTHTML), message);
    delay(500);
    ESP.restart();    
}

//---------------------------------------------------------------------------------
//-- 404
void handle_notFound(){
    String message = "File Not Found\n\n";
    message += "URI: ";
    message += webServer.uri();
    message += "\nMethod: ";
    message += (webServer.method() == HTTP_GET) ? "GET" : "POST";
    message += "\nArguments: ";
    message += webServer.args();
    message += "\n";
    for (uint8_t i = 0; i < webServer.args(); i++){
        message += " " + webServer.argName(i) + ": " + webServer.arg(i) + "\n";
    }
    webServer.send(404, FPSTR(kTEXTPLAIN), message);
}

//---------------------------------------------------------------------------------
MavESP8266Httpd::MavESP8266Httpd()
{

}

//---------------------------------------------------------------------------------
//-- Initialize
void
MavESP8266Httpd::begin(MavESP8266Update* updateCB_)
{
    updateCB = updateCB_;
    webServer.on("/",               handle_root);
    webServer.on("/getparameters",  handle_getParameters);
    webServer.on("/setparameters",  handle_setParameters);
    webServer.on("/getstatus",      handle_getStatus);
    webServer.on("/reboot",         handle_reboot);
    webServer.on("/setup",          handle_setup);
    webServer.on("/telemetry",      handle_telemetry);
    webServer.on("/mavlink_analyser", handle_mavlink_analyser);
    webServer.on("/mavlink_analyser/reboot", HTTP_POST, handle_reboot_autopilot);
    webServer.on("/api/foils",      handle_getFoils);
    webServer.on("/info.json",      handle_getJSysInfo);
    webServer.on("/status.json",    handle_getJSysStatus);
    webServer.on("/log.json",       handle_getJLog);
    webServer.on("/update",         handle_update);
    webServer.on("/upload",         HTTP_POST, handle_upload, handle_upload_status);
    webServer.onNotFound(           handle_notFound);
    webServer.begin();
}

//---------------------------------------------------------------------------------
//-- Initialize
void
MavESP8266Httpd::checkUpdates()
{
    webServer.handleClient();
}

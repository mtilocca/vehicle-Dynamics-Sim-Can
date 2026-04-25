// zephyr/src/http/http_html.hpp
// Static HTML string constants — no runtime data, edit freely.
// Included only by http_page.cpp.
#pragma once

// HTTP response header + HTML <head> open + <style> open tag.
// CSS content (from dashboard.css.inc) is injected immediately after this.
static const char kHtmlHead[] =
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: text/html; charset=utf-8\r\n"
    "Connection: close\r\n"
    "\r\n"
    "<!DOCTYPE html><html lang='en'><head>"
    "<meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Heavy-Duty Electric Vehicle &mdash; Simulator</title>"
    "<style>";

// Closes the <style> tag and <head>, opens <body>.
static const char kHtmlStyleClose[] = "</style></head><body>";

// Vehicle info card — hardcoded specs, never changes at runtime.
static const char kVehicleCard[] =
    "<div class='card'>"
    "<h2>Vehicle &mdash; Heavy-Duty Electric Vehicle</h2>"
    "<table><tr>"
    "<td>Mass</td><td>218 000 kg (218 t)</td>"
    "<td>Motor power</td><td>2 013 kW</td>"
    "<td>Motor torque</td><td>145 000 Nm</td>"
    "</tr><tr>"
    "<td>Battery</td><td>1 650 kWh</td>"
    "<td>Max speed</td><td>17.8 m/s (64 km/h)</td>"
    "<td>Gear ratio</td><td>28.0</td>"
    "</tr></table>"
    "</div>";

// JS auto-polling script — replaces the old <meta refresh>.
// Fetches /api/state every 5-8 seconds (randomised) and updates live spans.
static const char kHtmlFoot[] =
    "<script>"
    "function u(id,h){var e=document.getElementById(id);if(e)e.innerHTML=h;}"
    "function poll(){"
    "fetch('/api/state',{credentials:'include'})"
    ".then(function(r){return r.json();})"
    ".then(function(d){"
    "var h=Math.floor(d.uptime/3600),m=Math.floor(d.uptime%3600/60),s=d.uptime%60;"
    "u('up',(h<10?'0':'')+h+':'+(m<10?'0':'')+m+':'+(s<10?'0':'')+s);"
    "u('s-vx',d.vx.toFixed(3)+' m/s');"
    "u('s-vy',d.vy.toFixed(3)+' m/s');"
    "u('s-yaw',d.yaw.toFixed(2)+' °');"
    "u('s-yawr',d.yawr.toFixed(2)+' °/s');"
    "u('s-x',d.x.toFixed(2)+' m');"
    "u('s-y',d.y.toFixed(2)+' m');"
    "u('s-steer',d.steer.toFixed(2)+' °');"
    "u('s-soc','<span class=\\'val-hi\\'>'+d.soc.toFixed(1)+'%</span>');"
    "u('s-ofl',d.ofl.toFixed(2)+' rad/s');"
    "u('s-ofr',d.ofr.toFixed(2)+' rad/s');"
    "u('s-orl',d.orl.toFixed(2)+' rad/s');"
    "u('s-orr',d.orr.toFixed(2)+' rad/s');"
    "u('s-mu',d.mu.toFixed(2));"
    "var enc=d.en?'val-hi':'val-warn';"
    "u('c-en','<span class=\\''+enc+'\\'>'+( d.en?'ON':'OFF')+'</span>');"
    "u('c-gear',d.gear==='F'?'FORWARD':d.gear==='R'?'REVERSE':'NEUTRAL');"
    "u('c-torq',d.torq.toFixed(1)+' Nm');"
    "u('c-brk',d.brk.toFixed(2)+'%');"
    "u('c-steer',d.csteer.toFixed(2)+' °');"
    "u('can-tx',d.cantx);"
    "u('can-rx',d.canrx);"
    "u('can-to','<span class=\\''+( d.canto?'val-warn':'')+'\\'>'+d.canto+'</span>');"
    "u('can-lrx',d.lrx.toFixed(3)+' s');"
    "var hp=d.htot>0?Math.round(100*d.hu/d.htot):0;"
    "var hcls=hp>=80?'val-warn':hp>=60?'':'val-hi';"
    "u('res-hu','<span class=\\''+hcls+'\\'>'+d.hu+' B / '+d.htot+' B ('+hp+'%)</span>');"
    "u('res-hf',d.hf+' B');"
    "var lt=Math.round(d.loop/100);"
    "u('res-loop','<span class=\\''+( d.loop>10000?'val-warn':'val-hi')+'\\'>'+(lt/10).toFixed(1)+' ms</span>');"
    "var mqcls=d.mqc?'val-hi':'val-warn';"
    "u('mqtt-st','<span class=\\''+mqcls+'\\'>'+( d.mqc?'connected':'disconnected')+'</span>');"
    "u('mqtt-rx',d.mqrx);"
    "u('mm-hu',d.hu+' B used');"
    "u('mm-hf',d.hf+' B free');"
    "})"
    ".catch(function(){});"
    "setTimeout(poll,5000+Math.random()*3000);"
    "}"
    "setTimeout(poll,5000+Math.random()*3000);"
    "</script>"
    "</body></html>";

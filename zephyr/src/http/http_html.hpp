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
    "<meta http-equiv='refresh' content='30'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>XCMG XDE320 &mdash; Simulator</title>"
    "<style>";

// Closes the <style> tag and <head>, opens <body>.
static const char kHtmlStyleClose[] = "</style></head><body>";

// Vehicle info card — hardcoded specs, never changes at runtime.
static const char kVehicleCard[] =
    "<div class='card'>"
    "<h2>Vehicle &mdash; XCMG XDE320 Electric</h2>"
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

static const char kHtmlFoot[] = "</body></html>";

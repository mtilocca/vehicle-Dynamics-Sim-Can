import "./style.css";
import { mountDashboard } from "./dashboard";
import { mountOta } from "./ota";

// Firmware serves the same shell for /dash and /ota; route on the path.
if (location.pathname.startsWith("/ota")) mountOta();
else mountDashboard();

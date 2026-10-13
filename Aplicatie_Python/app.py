 # Proiect Licenta: Sistem IoT Smart Home bazat pe Bluetooth Mesh
 # Autor: Sfichi Alin-Ionuț
 # Aplicație: Interfața Grafică


import paho.mqtt.client as mqtt
import threading
import time
import json
import os
from flask import Flask, render_template_string, jsonify, request

app = Flask(__name__)

# --- CONFIGURARE MQTT ---
MQTT_BROKER = "127.0.0.1"
MQTT_PORT   = 1883
TOPIC_IN    = "mesh/in"    # Python -> Gateway
TOPIC_OUT   = "mesh/out"   # Gateway -> Python

# Baza de date locala
mesh_nodes            = {}
unprovisioned_devices = {}
prov_mode             = "MANUAL"
level_request_cooldown = {}
last_config_event     = {"addr": None, "ts": 0}

# --- AUTOMATIONS ---
AUTOMATIONS_FILE = "automations.json"
automations = []

def load_automations():
    global automations
    if os.path.exists(AUTOMATIONS_FILE):
        try:
            with open(AUTOMATIONS_FILE, "r") as f:
                automations = json.load(f)
        except:
            automations = []
    else:
        automations = []

def save_automations():
    clean = [{k: v for k, v in a.items() if k != "last_run"} for a in automations]
    with open(AUTOMATIONS_FILE, "w") as f:
        json.dump(clean, f)

load_automations()

# --- TIMER ENGINE ---
def run_timer_engine():
    """
    Thread separat care ruleaza la fiecare secunda si executa
    automatizarile de tip 'timer' (interval fix, fara trigger extern).
    """
    while True:
        try:
            now = time.time()
            for aut in automations:
                if not aut.get("enabled", True):
                    continue
                if aut.get("auto_type", "trigger") != "timer":
                    continue

                interval = int(aut.get("interval", 60))
                if now - aut.get("last_run", 0) >= interval:
                    aut["last_run"] = now
                    print(f"[TIMER] Executare regula: {aut['name']}")

                    action_int = int(aut["action_val"]) if aut.get("action_val") else 0
                    payload = {
                        "cmd":  aut["action_cmd"],
                        "addr": aut["action_addr"],
                    }
                    if aut["action_cmd"] in ("IR_SEND", "IR_LEARN"):
                        payload["slot"] = action_int
                    else:
                        payload["val"] = action_int

                    send_to_mqtt_json(payload)
        except Exception as e:
            import traceback
            print(f"[TIMER ERROR] {e}")
            traceback.print_exc()
        time.sleep(1)

threading.Thread(target=run_timer_engine, daemon=True).start()


def run_automation_engine(node_addr, key, val):
    try:
        for aut in automations:
            if not aut.get("enabled", True):                      continue
            # Automatizarile de tip timer nu se declanseaza din evenimente
            if aut.get("auto_type", "trigger") != "trigger":      continue
            if aut["trigger_addr"].lower() != node_addr.lower():  continue
            if aut["trigger_key"]          != key:                continue
            try:
                current = float(val)
                target  = float(aut["threshold"])
                cond    = aut["condition"]
                triggered = (
                    (cond == "above" and current > target) or
                    (cond == "below" and current < target) or
                    (cond == "equal" and current == target)
                )
            except:
                continue
            if triggered:
                now      = time.time()
                cooldown = int(aut.get("cooldown", 10))
                if now - aut.get("last_run", 0) > cooldown:
                    aut["last_run"] = now
                    print(f"[AUTO] Executare regula: {aut['name']}")

                    action_int = int(aut["action_val"]) if aut["action_val"] else 0
                    payload = {
                        "cmd":  aut["action_cmd"],
                        "addr": aut["action_addr"],
                    }
                    if aut["action_cmd"] in ("IR_SEND", "IR_LEARN"):
                        payload["slot"] = action_int
                    else:
                        payload["val"] = action_int

                    send_to_mqtt_json(payload)
    except Exception as e:
        import traceback
        print(f"[AUTO ERROR] {e}")
        traceback.print_exc()


# =============================================================================
# PARSER JSON (mesh/out)
# =============================================================================

def parse_json_message(msg_str):
    """
    Parseaza un mesaj JSON primit de la gateway si actualizeaza starea locala.
    Schema de tipuri:
      type=data      -> date nod (STATE, LEVEL, TEMP, HUMID, IR_STATUS)
      type=db_node   -> nod din baza de date (raspuns LIST_NODES)
      type=unprov    -> dispozitiv neprovizionat detectat
      type=event     -> eveniment sistem (CONFIG_DONE, NEW_NODE, etc.)
      type=ack       -> confirmare comanda
      type=error     -> eroare
      type=discovery -> informatii discovery composition data
      type=telemetry -> date telemetrie gateway
    """
    global mesh_nodes, unprovisioned_devices, prov_mode, last_config_event

    try:
        msg = json.loads(msg_str)
    except json.JSONDecodeError as e:
        print(f"[ERROR] JSON invalid de la gateway: {e} | raw: {msg_str}")
        return

    msg_type = msg.get("type")

    # ------------------------------------------------------------------
    if msg_type == "data":
        addr = msg.get("addr", "").lower()
        key  = msg.get("key", "")
        val  = msg.get("val", "")
        if not addr:
            return
        if addr not in mesh_nodes:
            mesh_nodes[addr] = {"addr": addr}
        node = mesh_nodes[addr]
        node["last_seen"] = time.strftime("%H:%M:%S")

        if key == "STATE":
            old_state = node.get("state")
            node["state"] = int(val)
            now = time.time()
            if (node.get("has_level") and
                    old_state != node["state"] and
                    now - level_request_cooldown.get(addr, 0) > 0.5):
                level_request_cooldown[addr] = now
                def _get_lvl(a=addr):
                    send_to_mqtt_json({"cmd": "GET_LVL", "addr": a})
                threading.Timer(0.5, _get_lvl).start()

        elif key == "LEVEL":
            node["level"] = int(val)

        elif key == "TEMP":
            node["temp"] = val

        elif key == "HUMID":
            node["humid"] = val

        elif key == "IR_STATUS":
            slot = msg.get("slot", "?")
            node["ir_feedback"] = f"{val} SLOT {slot}"

        run_automation_engine(addr, key, val)

    # ------------------------------------------------------------------
    elif msg_type == "db_node":
        addr = msg.get("addr", "").lower()
        if not addr:
            return
        if addr not in mesh_nodes:
            mesh_nodes[addr] = {"addr": addr, "state": None}
        mesh_nodes[addr].update({
            "has_onoff":         bool(msg.get("has_onoff",         False)),
            "has_onoff_client":  bool(msg.get("has_onoff_client",  False)),
            "has_level":         bool(msg.get("has_level",         False)),
            "has_sensor":        bool(msg.get("has_sensor",        False)),
            "has_ir":            bool(msg.get("has_ir",            False)),
            "last_seen":         time.strftime("%H:%M:%S"),
        })

    # ------------------------------------------------------------------
    elif msg_type == "unprov":
        uuid = msg.get("uuid", "")
        rssi = msg.get("rssi", 0)
        if uuid:
            unprovisioned_devices[uuid] = {"uuid": uuid, "rssi": rssi, "ts": time.time()}

    # ------------------------------------------------------------------
    elif msg_type == "event":
        name = msg.get("name", "")
        addr = msg.get("addr", "")

        if name == "CONFIG_DONE":
            last_config_event = {"addr": addr, "ts": time.time()}

        elif name in ("NODE_DELETED", "DB_CLEARED", "NODE_RESET"):
            if addr and addr.lower() in mesh_nodes:
                del mesh_nodes[addr.lower()]

        elif name == "DB_LIST_START":
            pass  

        elif name == "DB_LIST_END":
            pass

        elif name == "NODE_CONFIG_TIMEOUT":
            print(f"[WARN] Config timeout pentru nodul {addr}")

        elif name == "NEW_NODE":
            print(f"[INFO] Nod nou provizionat: {addr}")

    # ------------------------------------------------------------------
    elif msg_type == "ack":
        cmd = msg.get("cmd", "")
        if cmd == "PROV_MODE":
            prov_mode = msg.get("val", prov_mode)
        print(f"[ACK] {cmd}: {msg}")

    # ------------------------------------------------------------------
    elif msg_type == "error":
        print(f"[ERROR GW] {msg.get('msg')} addr={msg.get('addr', '-')}")

    # ------------------------------------------------------------------
    elif msg_type == "discovery":
        # Informatii suplimentare despre capabilitatile unui nod
        addr = msg.get("addr", "").lower()
        if addr and addr in mesh_nodes:
            mesh_nodes[addr].update({
                "has_onoff":        bool(msg.get("has_onoff",        False)),
                "has_onoff_client": bool(msg.get("has_onoff_client", False)),
                "has_level":        bool(msg.get("has_level",        False)),
                "has_sensor":       bool(msg.get("has_sensor",       False)),
                "has_ir":           bool(msg.get("has_ir",           False)),
            })

    # ------------------------------------------------------------------
    elif msg_type == "telemetry":
        key = msg.get("key", "")
        val = msg.get("val", "")
        print(f"[TELEMETRY] {key}={val}")

    else:
        print(f"[WARN] Tip mesaj necunoscut: {msg_type} | {msg}")


# --- Cleanup dispozitive neprovizionate vechi ---
def cleanup_scan_list():
    while True:
        now    = time.time()
        to_del = [u for u, d in unprovisioned_devices.items() if now - d["ts"] > 10]
        for u in to_del:
            del unprovisioned_devices[u]
        time.sleep(5)

threading.Thread(target=cleanup_scan_list, daemon=True).start()


# =============================================================================
# MQTT
# =============================================================================

def on_connect(client, userdata, flags, rc):
    client.subscribe(TOPIC_OUT)
    print(f"[MQTT] Conectat, subscris la {TOPIC_OUT}")

def on_message(client, userdata, msg):
    try:
        parse_json_message(msg.payload.decode("utf-8"))
    except Exception as e:
        print(f"[ERROR RX] {e}")
        import traceback
        traceback.print_exc()  # ← adaugă asta ca să vezi stack trace complet

mqtt_client = mqtt.Client()
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
try:
    mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
    mqtt_client.loop_start()
except Exception as e:
    print(f"[CRITICAL] Eroare conexiune MQTT: {e}")


def send_to_mqtt_json(payload: dict):
    """
    Serializeaza dict-ul ca JSON si il publica pe mesh/in.
    Aceasta este singura functie care scrie pe MQTT.
    """
    try:
        raw = json.dumps(payload, separators=(",", ":"))
        mqtt_client.publish(TOPIC_IN, raw)
        print(f"[MQTT TX] {raw}")
    except Exception as e:
        print(f"[ERROR TX] {e}")


# =============================================================================
# HTML TEMPLATE 
# =============================================================================

HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="ro">
<head>
    <meta charset="UTF-8">
    <title>Mesh Dashboard - Sfichi Alin</title>
    <script src="https://cdn.tailwindcss.com"></script>
    <style>
        @import url('https://fonts.googleapis.com/css2?family=Inter:wght@400;700;900&display=swap');
        body { background: #0f172a; color: #f1f5f9; font-family: 'Inter', sans-serif; overflow-x: hidden; }
        .node-card { background: #1e293b; border: 2px solid #334155; border-radius: 1rem; padding: 1.5rem; transition: border-color 0.1s; }
        .node-card:hover { border-color: #6366f1; }
        .btn { font-weight: 800; text-transform: uppercase; font-size: 0.7rem; cursor: pointer; }
        .btn:active { transform: scale(0.95); }
        input[type="range"] { accent-color: #6366f1; cursor: pointer; }
        .modal { background: rgba(15, 23, 42, 0.9); backdrop-filter: blur(8px); }
        .status-banner { animation: slideIn 0.3s ease-out; }
        @keyframes slideIn { from { transform: translateY(-20px); opacity: 0; } to { transform: translateY(0); opacity: 1; } }
        .switch { position: relative; display: inline-block; width: 34px; height: 20px; }
        .switch input { opacity: 0; width: 0; height: 0; }
        .slider { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background-color: #334155; transition: .4s; border-radius: 20px; }
        .slider:before { position: absolute; content: ""; height: 14px; width: 14px; left: 3px; bottom: 3px; background-color: white; transition: .4s; border-radius: 50%; }
        input:checked + .slider { background-color: #6366f1; }
        input:checked + .slider:before { transform: translateX(14px); }
    </style>
</head>
<body class="p-4 md:p-10">
    <div class="max-w-6xl mx-auto">
        <header class="flex flex-col md:flex-row justify-between items-center mb-12 gap-6">
            <div>
                <h1 class="text-3xl font-black text-indigo-500 tracking-tighter uppercase">Mesh Control Center</h1>
                <p class="text-slate-500 font-medium tracking-tight uppercase text-[10px]">Proiect licență — Sfichi Alin-Ionuț</p>
            </div>
            <div class="flex gap-3">
                <button onclick="openModal('autoModal')" class="bg-amber-500 hover:bg-amber-400 text-black px-6 py-4 rounded-2xl font-black shadow-lg uppercase text-xs">Automatizări</button>
                <button onclick="openProv()" class="bg-emerald-600 hover:bg-emerald-500 text-white px-6 py-4 rounded-2xl font-black shadow-lg uppercase text-xs">Provisioning</button>
                <button onclick="reloadDB()" class="bg-indigo-600 hover:bg-indigo-500 text-white px-6 py-4 rounded-2xl font-black shadow-lg uppercase text-xs">Reload NODES</button>
                <button onclick="openDBMenu()" class="bg-slate-700 hover:bg-rose-600 text-white px-6 py-4 rounded-2xl font-black shadow-lg uppercase text-xs">⚙ DB</button>
            </div>
        </header>

        <div id="grid" class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-8 text-white">
            <div class="col-span-full text-center py-20 text-slate-600 font-bold italic text-lg text-indigo-400 animate-pulse">Sincronizare rețea...</div>
        </div>
    </div>

    <!-- MODAL AUTOMATIZĂRI -->
    <div id="autoModal" class="hidden fixed inset-0 modal flex items-center justify-center p-4">
        <div class="bg-slate-800 border border-slate-700 w-full max-w-5xl rounded-3xl p-8 shadow-2xl max-h-[90vh] overflow-y-auto">
            <div class="flex justify-between items-center mb-8">
                <h2 class="text-2xl font-black text-white uppercase">Management Automatizări</h2>
                <button onclick="closeModal('autoModal')" class="text-slate-400 hover:text-white text-3xl">&times;</button>
            </div>
            <div class="grid grid-cols-1 lg:grid-cols-3 gap-8">
                <div class="lg:col-span-1 bg-slate-900/50 p-6 rounded-2xl border border-slate-700">
                    <h3 id="formTitle" class="text-indigo-400 font-black text-xs uppercase mb-4 tracking-widest">Regulă Nouă</h3>
                    <div class="space-y-4">
                        <!-- Tip automatizare -->
                        <div class="flex gap-2">
                            <label class="flex-1 flex items-center justify-center gap-2 bg-slate-800 border border-slate-700 p-3 rounded-xl cursor-pointer text-xs font-black uppercase has-[:checked]:border-indigo-500 has-[:checked]:text-indigo-400">
                                <input type="radio" name="autoType" value="trigger" checked onchange="onAutoTypeChange()"> Trigger
                            </label>
                            <label class="flex-1 flex items-center justify-center gap-2 bg-slate-800 border border-slate-700 p-3 rounded-xl cursor-pointer text-xs font-black uppercase has-[:checked]:border-amber-500 has-[:checked]:text-amber-400">
                                <input type="radio" name="autoType" value="timer" onchange="onAutoTypeChange()"> Timer
                            </label>
                        </div>

                        <input type="text" id="autoName" placeholder="Nume (ex: Auto AC)" class="w-full bg-slate-800 p-3 rounded-xl text-xs border border-slate-700 outline-none focus:border-indigo-500">

                        <!-- Câmpuri TRIGGER -->
                        <div id="triggerFields">
                            <p class="text-[10px] font-bold text-slate-500 uppercase">Dacă nodul:</p>
                            <select id="autoTrigAddr" class="w-full bg-slate-800 p-3 rounded-xl text-xs border border-slate-700 mt-2"></select>
                            <select id="autoTrigKey" class="w-full bg-slate-800 p-3 rounded-xl text-xs border border-slate-700 mt-2">
                                <option value="TEMP">Temperatura</option>
                                <option value="HUMID">Umiditatea</option>
                                <option value="STATE">Starea (ON/OFF)</option>
                            </select>
                            <div class="flex gap-2 mt-2">
                                <select id="autoCond" class="bg-slate-800 p-3 rounded-xl text-xs border border-slate-700">
                                    <option value="below">&lt;</option>
                                    <option value="above">&gt;</option>
                                    <option value="equal">=</option>
                                </select>
                                <input type="number" id="autoThresh" placeholder="Valoare" class="w-full bg-slate-800 p-3 rounded-xl text-xs border border-slate-700">
                            </div>
                            <div class="pt-2">
                                <p class="text-[10px] font-bold text-slate-500 uppercase mb-2">Cooldown (secunde):</p>
                                <input type="number" id="autoCooldown" value="10" min="1" class="w-full bg-slate-800 p-3 rounded-xl text-xs border border-slate-700">
                            </div>
                        </div>

                        <!-- Câmpuri TIMER -->
                        <div id="timerFields" style="display:none">
                            <p class="text-[10px] font-bold text-amber-500 uppercase mb-2">Repetă la fiecare (secunde):</p>
                            <input type="number" id="autoInterval" value="60" min="1" class="w-full bg-slate-800 p-3 rounded-xl text-xs border border-amber-600/40 outline-none focus:border-amber-500">
                            <p class="text-[9px] text-slate-500 italic mt-1">Ex: 60 = la fiecare minut, 3600 = la fiecare oră</p>
                        </div>

                        <p class="text-[10px] font-bold text-slate-500 uppercase pt-2">Execută pe:</p>
                        <select id="autoActAddr" class="w-full bg-slate-800 p-3 rounded-xl text-xs border border-slate-700"></select>
                        <select id="autoActCmd" class="w-full bg-slate-800 p-3 rounded-xl text-xs border border-slate-700">
                            <option value="ON">Aprinde (ON)</option>
                            <option value="OFF">Stinge (OFF)</option>
                            <option value="IR_SEND">Trimite IR Slot</option>
                        </select>
                        <input type="number" id="autoActVal" placeholder="Slot/Val" class="w-full bg-slate-800 p-3 rounded-xl text-xs border border-slate-700" value="0">
                        <div class="flex gap-2 pt-4">
                            <button id="autoCancelBtn" onclick="resetAutoForm()" class="hidden flex-1 bg-slate-700 p-4 rounded-xl font-black text-xs uppercase">Anulează</button>
                            <button id="autoSaveBtn" onclick="saveAutomation()" class="flex-[2] bg-indigo-600 p-4 rounded-xl font-black text-xs uppercase hover:bg-indigo-500 transition">Salvează</button>
                        </div>
                    </div>
                </div>
                <div class="lg:col-span-2">
                    <h3 class="text-slate-500 font-black text-xs uppercase mb-4 tracking-widest">Reguli Active</h3>
                    <div id="automationsList" class="space-y-3"></div>
                </div>
            </div>
        </div>
    </div>

    <!-- MODAL PROVISIONING -->
    <div id="provModal" class="hidden fixed inset-0 modal z-50 flex items-center justify-center p-4">
        <div class="bg-slate-800 border border-slate-700 w-full max-w-xl rounded-3xl p-8 shadow-2xl relative">
            <div class="flex justify-between items-center mb-8">
                <h2 class="text-xl font-black text-white uppercase">Management Provizionare</h2>
                <button onclick="closeProv()" class="text-slate-400 hover:text-white text-2xl font-bold">&times;</button>
            </div>
            <div id="provFeedback" class="hidden mb-6 p-4 rounded-2xl border status-banner"></div>
            <div class="mb-8 p-4 bg-slate-900/50 rounded-2xl border border-slate-700 flex justify-between items-center">
                <div><p class="text-[10px] font-black text-indigo-400 uppercase tracking-widest">Mod Curent</p><p id="currentMode" class="font-bold text-lg">MANUAL</p></div>
                <div class="flex gap-2">
                    <button onclick="setProvMode(1)" class="bg-slate-700 hover:bg-indigo-600 px-4 py-2 rounded-xl text-[10px] font-black uppercase">Auto</button>
                    <button onclick="setProvMode(0)" class="bg-slate-700 hover:bg-rose-600 px-4 py-2 rounded-xl text-[10px] font-black uppercase">Manual</button>
                </div>
            </div>
            <h3 class="text-[10px] font-black text-slate-500 uppercase tracking-widest mb-4">Dispozitive detectate</h3>
            <div id="scanList" class="space-y-3 max-h-60 overflow-y-auto pr-2"></div>
        </div>
    </div>

    <!-- MODAL CONFIRM -->
    <div id="confirmModal" class="hidden fixed inset-0 modal z-50 flex items-center justify-center p-4">
        <div class="bg-slate-800 border border-rose-500/30 w-full max-w-md rounded-3xl p-8 shadow-2xl">
            <div class="text-center mb-6">
                <div class="text-4xl mb-3">!!</div>
                <h2 class="text-xl font-black text-rose-400 uppercase mb-2" id="confirmTitle">Confirmare</h2>
                <p class="text-slate-400 text-sm" id="confirmMsg"></p>
            </div>
            <div class="flex gap-3">
                <button onclick="closeConfirm()" class="btn flex-1 bg-slate-700 hover:bg-slate-600 text-white py-3 rounded-2xl">Anulează</button>
                <button onclick="executeConfirm()" class="btn flex-1 bg-rose-600 hover:bg-rose-500 text-white py-3 rounded-2xl">Confirmă</button>
            </div>
        </div>
    </div>

    <!-- MODAL DB MENU -->
    <div id="dbModal" class="hidden fixed inset-0 modal z-50 flex items-center justify-center p-4">
        <div class="bg-slate-800 border border-slate-700 w-full max-w-sm rounded-3xl p-8 shadow-2xl">
            <div class="flex justify-between items-center mb-6">
                <h2 class="text-xl font-black text-white uppercase">Gestionare DB</h2>
                <button onclick="closeDBMenu()" class="text-slate-400 hover:text-white text-2xl font-bold">&times;</button>
            </div>
            <div class="space-y-3">
                <button onclick="confirmAction('delete_db', null)" class="w-full btn bg-rose-600/20 text-rose-400 border border-rose-600/30 py-4 rounded-2xl hover:bg-rose-600 hover:text-white transition text-left px-5">
                    <span class="block font-black uppercase text-xs">Șterge toată baza de date</span>
                    <span class="block text-[10px] text-slate-500 mt-1 normal-case font-normal">Elimină toate nodurile din memorie și NVS</span>
                </button>
            </div>
        </div>
    </div>

    <script>
        let meshNodes = {};
        let selectedSlots = {};
        let polledNodes = new Set();
        let lastSensorPoll = {};
        let lastHash = "";
        let isProvisioningInProgress = false;
        let pendingAction = null;
        let pendingAddr   = null;
        let editingIndex  = null;

        function openModal(id)  { document.getElementById(id).classList.remove('hidden'); if(id==='autoModal') updateAutoDropdowns(); }
        function closeModal(id) { document.getElementById(id).classList.add('hidden'); }
        function updateAutoDropdowns() {
            const opts = Object.values(meshNodes).map(n => `<option value="${n.addr}">${n.addr.toUpperCase()}</option>`).join('');
            document.getElementById('autoTrigAddr').innerHTML = opts;
            document.getElementById('autoActAddr').innerHTML  = opts;
        }

        async function saveAutomation() {
            const autoType = document.querySelector('input[name="autoType"]:checked').value;
            const data = {
                auto_type:    autoType,
                name:         document.getElementById('autoName').value,
                action_addr:  document.getElementById('autoActAddr').value,
                action_cmd:   document.getElementById('autoActCmd').value,
                action_val:   document.getElementById('autoActVal').value,
                enabled:      true
            };
            if (autoType === 'trigger') {
                data.trigger_addr = document.getElementById('autoTrigAddr').value;
                data.trigger_key  = document.getElementById('autoTrigKey').value;
                data.condition    = document.getElementById('autoCond').value;
                data.threshold    = document.getElementById('autoThresh').value;
                data.cooldown     = document.getElementById('autoCooldown').value;
            } else {
                data.interval = document.getElementById('autoInterval').value;
            }
            const url    = editingIndex !== null ? `/api/automations/${editingIndex}` : '/api/automations';
            const method = editingIndex !== null ? 'PUT' : 'POST';
            await fetch(url, { method, headers: {'Content-Type':'application/json'}, body: JSON.stringify(data) });
            resetAutoForm();
            updateUI();
        }
        
        function onAutoTypeChange() {
            const isTrigger = document.querySelector('input[name="autoType"]:checked').value === 'trigger';
            document.getElementById('triggerFields').style.display  = isTrigger ? '' : 'none';
            document.getElementById('timerFields').style.display    = isTrigger ? 'none' : '';
        }

        function editAuto(index) {
            fetch('/api/automations').then(r => r.json()).then(autos => {
                const a = autos[index];
                editingIndex = index;
                document.getElementById('autoName').value     = a.name;
                document.getElementById('autoTrigAddr').value = a.trigger_addr;
                document.getElementById('autoTrigKey').value  = a.trigger_key;
                document.getElementById('autoCond').value     = a.condition;
                document.getElementById('autoThresh').value   = a.threshold;
                document.getElementById('autoCooldown').value = a.cooldown || 10;
                document.getElementById('autoActAddr').value  = a.action_addr;
                document.getElementById('autoActCmd').value   = a.action_cmd;
                document.getElementById('autoActVal').value   = a.action_val;
                document.getElementById('formTitle').innerText    = "Editare Regulă";
                document.getElementById('autoSaveBtn').innerText  = "Actualizează";
                document.getElementById('autoCancelBtn').classList.remove('hidden');
            });
        }

        function resetAutoForm() {
            editingIndex = null;
            ['autoName','autoThresh','autoActVal'].forEach(id => document.getElementById(id).value = '');
            document.getElementById('autoCooldown').value      = '10';
            document.getElementById('formTitle').innerText     = 'Regulă Nouă';
            document.getElementById('autoSaveBtn').innerText   = 'Salvează';
            document.getElementById('autoCancelBtn').classList.add('hidden');
        }

        async function toggleAuto(index) { await fetch(`/api/automations/${index}/toggle`, {method:'PATCH'}); updateUI(); }
        async function deleteAuto(index) { await fetch(`/api/automations/${index}`, {method:'DELETE'}); updateUI(); }

        function confirmAction(action, addr) {
            closeDBMenu();
            pendingAction = action; pendingAddr = addr;
            const title = document.getElementById('confirmTitle');
            const msg   = document.getElementById('confirmMsg');
            if (action === 'delete_db') {
                title.innerText = 'Șterge toată baza de date?';
                msg.innerHTML   = 'Toate nodurile vor fi eliminate din memorie și NVS.<br><br><span class="text-rose-400 font-bold">!! Va trebui să resetați hardware fiecare nod pentru a relua provisioningul.</span>';
            } else if (action === 'delete_node') {
                title.innerText = `Șterge nodul ${addr}?`;
                msg.innerHTML   = `Nodul <span class="font-mono text-white">${addr}</span> va fi eliminat din baza de date.<br><br><span class="text-rose-400 font-bold">!! Va trebui să resetați hardware nodul pentru a relua provisioningul.</span>`;
            }
            document.getElementById('confirmModal').classList.remove('hidden');
        }

        function executeConfirm() {
            closeConfirm();
            if (pendingAction === 'delete_db') {
                run('DELETE_DB', null);
                setTimeout(() => reloadDB(), 500);
            } else if (pendingAction === 'delete_node' && pendingAddr) {
                run('DELETE_NODE', pendingAddr);
                setTimeout(() => reloadDB(), 500);
            }
            pendingAction = null; pendingAddr = null;
        }

        function closeConfirm()  { document.getElementById('confirmModal').classList.add('hidden'); }
        function openDBMenu()    { document.getElementById('dbModal').classList.remove('hidden'); }
        function closeDBMenu()   { document.getElementById('dbModal').classList.add('hidden'); }

        function toggleNodeMenu(addr) {
            const menu = document.getElementById('nodemenu_' + addr);
            document.querySelectorAll('[id^="nodemenu_"]').forEach(m => { if (m.id !== 'nodemenu_' + addr) m.classList.add('hidden'); });
            menu.classList.toggle('hidden');
        }
        function closeNodeMenu(addr) { document.getElementById('nodemenu_' + addr).classList.add('hidden'); }
        document.addEventListener('click', function(e) {
            if (!e.target.closest('[id^="nodemenu_"]') && !e.target.closest('button[onclick^="toggleNodeMenu"]'))
                document.querySelectorAll('[id^="nodemenu_"]').forEach(m => m.classList.add('hidden'));
        });

        function openProv()  { document.getElementById('provModal').classList.remove('hidden'); }
        function closeProv() { if(isProvisioningInProgress) return; document.getElementById('provModal').classList.add('hidden'); document.getElementById('provFeedback').classList.add('hidden'); }

        // Toate comenzile trimise catre gateway folosesc acelasi format JSON
        async function run(cmd, addr, extra = {}, isAuto = false) {
            const payload = { cmd };
            if (addr) payload.addr = addr;
            Object.assign(payload, extra);

            await fetch('/api/send', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify(payload)
            });

            if (!isAuto && addr) {
                const node = meshNodes[addr];
                if (!node) return;
                if (cmd === 'ON' || cmd === 'OFF') {
                    setTimeout(() => run('GET_ONOFF', addr, {}, true), 200);
                }
                if (cmd === 'LVL') {
                    setTimeout(() => run('GET_LVL', addr, {}, true), 200);
                    if (node.has_onoff) setTimeout(() => run('GET_ONOFF', addr, {}, true), 400);
                }
            }
        }

        function setProvMode(m) {
            fetch('/api/prov_mode', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({mode: m})});
        }

        async function startProv(uuid) {
            if (isProvisioningInProgress) return;
            isProvisioningInProgress = true;
            const feedbackEl = document.getElementById('provFeedback');
            feedbackEl.className = "mb-6 p-4 rounded-2xl border status-banner bg-indigo-500/10 border-indigo-500/30 text-indigo-400 font-bold text-sm text-center";
            feedbackEl.innerHTML = `<span class="animate-pulse">Se adaugă dispozitivul... Vă rugăm așteptați.</span>`;
            feedbackEl.classList.remove('hidden');
            await fetch('/api/start_prov', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({uuid})});
            const startTime = Date.now();
            const checkInterval = setInterval(async () => {
                const res  = await fetch('/api/prov_status');
                const data = await res.json();
                const eventAge = Date.now() - (data.last_event_ts * 1000);
                if (data.last_event_addr && eventAge < 5000) {
                    clearInterval(checkInterval);
                    showStatus("ADĂUGARE CU SUCCES!", `Nodul a primit adresa ${data.last_event_addr}.`, "emerald");
                    isProvisioningInProgress = false;
                } else if (Date.now() - startTime > 7000) {
                    clearInterval(checkInterval);
                    showStatus("EROARE PROVIZIONARE!", "Timpul a expirat. Resetați dispozitivul și reîncercați.", "rose");
                    isProvisioningInProgress = false;
                }
            }, 500);
        }

        function showStatus(title, msg, color) {
            const el = document.getElementById('provFeedback');
            el.className = `mb-6 p-4 rounded-2xl border status-banner bg-${color}-500/10 border-${color}-500/30 text-${color}-400 text-center`;
            el.innerHTML = `<p class="font-black text-xs uppercase mb-1">${title}</p><p class="text-[11px] opacity-80">${msg}</p>`;
        }

        function bindSwitch(switchAddr) {
            const target = document.getElementById('target_' + switchAddr).value.trim();
            if (!target.startsWith('0x')) { alert("Format invalid (ex: 0x0006)"); return; }
            run('SET_PUB', switchAddr, { target });
        }

        function reloadDB() {
            polledNodes.clear();
            lastSensorPoll = {};
            lastHash = "";
            fetch('/api/reload');
        }

        function updateUI() {
            fetch('/api/nodes').then(r => r.json()).then(nodes => {
                meshNodes = nodes;
                const grid      = document.getElementById('grid');
                const nodeArray = Object.values(nodes);

                if (nodeArray.length === 0) {
                    grid.innerHTML = '<div class="col-span-full text-center py-20 text-slate-600 font-bold italic text-lg text-indigo-400 animate-pulse">Aștept sincronizare noduri...</div>';
                    lastHash = "";
                    return;
                }

                const now = Date.now();
                let currentHash = "";

                nodeArray.forEach(n => {
                    currentHash += n.addr + n.state + n.level + n.temp + n.humid + n.ir_feedback + n.has_onoff + n.has_onoff_client;

                    if (!polledNodes.has(n.addr)) {
                        polledNodes.add(n.addr);
                        let d = 600;
                        if (n.has_onoff)  { setTimeout(() => run('GET_ONOFF',  n.addr, {}, true), d); d += 600; }
                        if (n.has_sensor) { setTimeout(() => run('GET_SENSOR', n.addr, {}, true), d); lastSensorPoll[n.addr] = now; }
                    }
                    if (n.has_sensor && (now - (lastSensorPoll[n.addr] || 0) > 30000)) {
                        run('GET_SENSOR', n.addr, {}, true);
                        lastSensorPoll[n.addr] = now;
                    }
                });

                if (lastHash !== currentHash) {
                    grid.innerHTML = nodeArray.map(n => {
                        let html = `<div class="node-card">
                            <div class="flex justify-between items-center mb-6">
                                <span class="bg-indigo-500/10 text-indigo-400 border border-indigo-500/20 px-3 py-1 rounded-lg text-xs font-black font-mono tracking-widest">${n.addr.toUpperCase()}</span>
                                <div class="flex items-center gap-2">
                                    <span class="node-time text-[10px] text-slate-500 font-bold" data-addr="${n.addr}">${n.last_seen || ''}</span>
                                    <div class="relative">
                                        <button onclick="toggleNodeMenu('${n.addr}')" class="text-slate-500 hover:text-white w-8 h-8 flex items-center justify-center rounded-xl hover:bg-slate-700 font-black text-lg">⋮</button>
                                        <div id="nodemenu_${n.addr}" class="hidden absolute right-0 top-9 bg-slate-900 border border-slate-700 rounded-2xl shadow-2xl z-10 w-52 overflow-hidden">
                                            <button onclick="closeNodeMenu('${n.addr}'); confirmAction('delete_node', '${n.addr}')"
                                                class="w-full text-left px-4 py-3 text-rose-400 hover:bg-rose-600/20 text-xs font-black uppercase transition">
                                                Șterge nod
                                            </button>
                                        </div>
                                    </div>
                                </div>
                            </div>`;

                        if (n.has_onoff_client) {
                            html += `<div class="mb-6 pb-6 border-b border-slate-700/50">
                                <div class="mb-4 flex justify-between items-center">
                                    <span class="text-[10px] font-black text-amber-500 uppercase tracking-widest">Link Direct</span>
                                    <span class="text-[8px] text-slate-600 font-bold">CLIENT</span>
                                </div>
                                <div class="flex gap-2">
                                    <input type="text" id="target_${n.addr}" placeholder="0x0006"
                                        class="bg-slate-800 border border-slate-700 rounded-xl p-2 text-[10px] w-full focus:border-amber-500 outline-none font-mono">
                                    <button onclick="bindSwitch('${n.addr}')" class="btn bg-amber-600 px-4 py-2 rounded-xl text-white hover:bg-amber-500 transition shadow-lg shadow-amber-900/20">BIND</button>
                                </div>
                                <p class="text-[9px] text-slate-500 mt-2 italic text-center">Configurați nodul bec controlat</p>
                            </div>`;
                        }

                        if (n.has_onoff) {
                            const st = n.state === 1 ? 'PORNIT' : (n.state === 0 ? 'OPRIT' : 'CITIRE...');
                            const cl = n.state === 1 ? 'text-emerald-400' : (n.state === 0 ? 'text-slate-500' : 'text-indigo-400 animate-pulse');
                            html += `<div class="mb-6 pb-6 border-b border-slate-700/50">
                                <div class="flex justify-between items-center mb-4 text-[10px] font-black text-slate-500 uppercase tracking-widest">
                                    <span>Lumină</span>
                                    <button onclick="run('GET_ONOFF', '${n.addr}', {}, true)" class="text-indigo-400 font-bold uppercase underline">Refresh</button>
                                </div>
                                <div class="flex gap-2">
                                    <button onclick="run('ON',  '${n.addr}')" class="btn flex-1 bg-emerald-600/20 text-emerald-400 border border-emerald-600/30 py-3 rounded-xl hover:bg-emerald-600 hover:text-white transition">ON</button>
                                    <button onclick="run('OFF', '${n.addr}')" class="btn flex-1 bg-rose-600/20 text-rose-400 border border-rose-600/30 py-3 rounded-xl hover:bg-rose-600 hover:text-white transition">OFF</button>
                                </div>
                                <p class="mt-3 text-center text-[11px] font-black ${cl}">STATUS: ${st}</p>
                            </div>`;
                        }

                        if (n.has_level) {
                            const rawLevel = (n.level !== undefined && n.level !== null) ? n.level : -32768;
                            const pct = Math.round((rawLevel + 32768) / 65535 * 100);
                            html += `<div class="mb-6 pb-6 border-b border-slate-700/50">
                                <div class="flex justify-between items-center mb-3 text-[10px] font-black text-slate-500 uppercase tracking-widest">
                                    <span>Intensitate</span>
                                    <button onclick="run('GET_LVL', '${n.addr}', {}, true)" class="text-indigo-400 font-bold uppercase underline">Refresh</button>
                                </div>
                                <div class="flex justify-between items-center text-[11px] font-black text-indigo-300 mb-3 uppercase">
                                    <span>Nivel: <span id="lvl_pct_${n.addr}">${pct}</span>%</span>
                                    <span class="text-[9px] text-slate-600 font-mono">raw: ${rawLevel}</span>
                                </div>
                                <input type="range" min="0" max="100" value="${pct}"
                                    oninput="document.getElementById('lvl_pct_${n.addr}').innerText = this.value"
                                    onchange="run('LVL', '${n.addr}', {val: Math.round(parseInt(this.value) / 100 * 65535 - 32768)})"
                                    class="w-full h-2 bg-slate-700 rounded-lg appearance-none">
                                <div class="flex justify-between text-[9px] text-slate-600 font-bold mt-1 uppercase">
                                    <span>0%</span><span>50%</span><span>100%</span>
                                </div>
                            </div>`;
                        }

                        if (n.has_sensor) {
                            html += `<div class="mb-6 pb-6 border-b border-slate-700/50 text-center">
                                <div class="flex justify-between items-center mb-4 text-[10px] font-black text-slate-500 uppercase tracking-widest">
                                    <span>Date Mediu</span>
                                    <button onclick="run('GET_SENSOR', '${n.addr}', {}, true)" class="text-indigo-400 font-bold uppercase underline">Request</button>
                                </div>
                                <div class="grid grid-cols-2 gap-3 font-black text-center text-xs">
                                    <div class="bg-slate-800/50 p-3 rounded-2xl border border-slate-700/50 text-orange-400"><span class="text-[9px] block uppercase mb-1 font-bold">Temperatura</span>${n.temp || '--'}°C</div>
                                    <div class="bg-slate-800/50 p-3 rounded-2xl border border-slate-700/50 text-cyan-400"><span class="text-[9px] block uppercase mb-1 font-bold">Umiditate</span>${n.humid || '--'}%</div>
                                </div>
                            </div>`;
                        }

                        if (n.has_ir) {
                            const sel = selectedSlots[n.addr] || 0;
                            html += `<div>
                                <div class="flex justify-between items-center mb-2 font-black text-slate-500 uppercase text-[10px] tracking-widest">
                                    <span>IR Controller</span>
                                    <span class="text-amber-500 animate-pulse">${n.ir_feedback || 'IDLE'}</span>
                                </div>
                                <div class="grid grid-cols-5 gap-1 mb-4">
                                    ${[0,1,2,3,4].map(i => `<button onclick="selectedSlots['${n.addr}']=${i}; lastHash=''; updateUI();" class="py-2 rounded-lg text-[10px] font-black transition ${sel==i ? 'bg-amber-500 text-black shadow-md' : 'bg-slate-800 text-slate-500'}">S${i}</button>`).join('')}
                                </div>
                                <div class="flex gap-2">
                                    <button onclick="run('IR_LEARN', '${n.addr}', {slot: ${sel}})" class="btn flex-1 bg-amber-600/20 text-amber-500 border border-amber-600/30 py-2.5 rounded-xl hover:bg-amber-600 hover:text-white transition">Learn</button>
                                    <button onclick="run('IR_SEND',  '${n.addr}', {slot: ${sel}})" class="btn flex-1 bg-blue-600/20 text-blue-500 border border-blue-600/30 py-2.5 rounded-xl hover:bg-blue-600 hover:text-white transition">Send</button>
                                </div>
                            </div>`;
                        }

                        return html + `</div>`;
                    }).join('');
                    lastHash = currentHash;
                } else {
                    document.querySelectorAll('.node-time').forEach(el => {
                        const addr = el.getAttribute('data-addr');
                        if (nodes[addr]) el.innerText = nodes[addr].last_seen;
                    });
                }
            });

            fetch('/api/unprovisioned').then(r => r.json()).then(data => {
                document.getElementById('currentMode').innerText = data.mode;
                const list = document.getElementById('scanList');
                const devs = Object.values(data.devices);
                if (devs.length === 0) {
                    list.innerHTML = '<p class="text-center text-slate-600 italic py-10 text-xs font-bold uppercase tracking-widest">Căutare dispozitive noi...</p>';
                    return;
                }
                list.innerHTML = devs.map(d => `
                    <div class="bg-slate-900 border border-slate-700 p-4 rounded-2xl flex justify-between items-center">
                        <div class="flex flex-col">
                            <span class="text-[10px] font-mono text-white font-bold">${d.uuid.toUpperCase()}</span>
                            <span class="text-[8px] font-black text-indigo-400 uppercase mt-1">RSSI: ${d.rssi} dBm</span>
                        </div>
                        <button onclick="startProv('${d.uuid}')" class="btn bg-indigo-600 text-white px-4 py-2 rounded-xl hover:bg-indigo-500 ${isProvisioningInProgress ? 'opacity-50 cursor-not-allowed' : ''}">Add</button>
                    </div>`).join('');
            });

            fetch('/api/automations').then(r => r.json()).then(autos => {
                document.getElementById('automationsList').innerHTML = autos.map((a, i) => {
                const isTimer = (a.auto_type || 'trigger') === 'timer';
                const badge = isTimer
                    ? `<span class="text-[8px] font-black bg-amber-500/20 text-amber-400 px-2 py-0.5 rounded-full uppercase">Timer</span>`
                    : `<span class="text-[8px] font-black bg-indigo-500/20 text-indigo-400 px-2 py-0.5 rounded-full uppercase">Trigger</span>`;
                const detail = isTimer
                    ? `<p class="text-[10px] text-slate-500 mt-1 uppercase">EVERY <span class="text-amber-400 font-bold">${a.interval}s</span> → <span class="text-emerald-400 font-bold">${a.action_cmd}</span> → ${a.action_addr}</p>`
                    : `<p class="text-[10px] text-slate-500 mt-1 uppercase">IF <span class="text-indigo-400 font-bold">${a.trigger_addr}</span> ${a.trigger_key} ${a.condition} ${a.threshold} THEN <span class="text-emerald-400 font-bold">${a.action_cmd}</span> → ${a.action_addr}</p><p class="text-[8px] text-slate-600 mt-1 font-bold tracking-widest uppercase">Cooldown: ${a.cooldown}s</p>`;
                return `
                    <div class="bg-slate-900 p-5 rounded-2xl border border-slate-700 flex justify-between items-center ${a.enabled ? '' : 'opacity-40'}">
                        <div class="flex-1">
                            <div class="flex items-center gap-3">
                                <p class="font-black text-white text-sm">${a.name}</p>
                                ${badge}
                                <label class="switch"><input type="checkbox" ${a.enabled ? 'checked' : ''} onclick="toggleAuto(${i})"><span class="slider"></span></label>
                            </div>
                            ${detail}
                        </div>
                        <div class="flex gap-2">
                            <button onclick="editAuto(${i})"   class="bg-slate-800 p-2 rounded-lg text-indigo-400 hover:bg-indigo-600/20 transition">Edit</button>
                            <button onclick="deleteAuto(${i})" class="bg-slate-800 p-2 rounded-lg text-rose-500 hover:bg-rose-600/20 transition">Del</button>
                        </div>
                    </div>`;
            }).join('');
            });
        }

        setInterval(updateUI, 1000);
        updateUI();
    </script>
</body>
</html>
"""

# =============================================================================
# RUTE API
# =============================================================================

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/api/nodes')
def get_nodes():
    return jsonify(mesh_nodes)

@app.route('/api/unprovisioned')
def get_unprov():
    return jsonify({"mode": prov_mode, "devices": unprovisioned_devices})

@app.route('/api/prov_status')
def get_prov_status():
    return jsonify({"last_event_addr": last_config_event["addr"], "last_event_ts": last_config_event["ts"]})

@app.route('/api/prov_mode', methods=['POST'])
def api_prov_mode():
    mode = request.json.get('mode')
    send_to_mqtt_json({"cmd": "SET_PROV_MODE", "val": int(mode)})
    return jsonify({"status": "ok"})

@app.route('/api/start_prov', methods=['POST'])
def api_start_prov():
    uuid = request.json.get('uuid')
    send_to_mqtt_json({"cmd": "START_PROV", "uuid": uuid})
    return jsonify({"status": "ok"})

@app.route('/api/reload')
def reload_nodes():
    global mesh_nodes
    mesh_nodes = {}
    send_to_mqtt_json({"cmd": "LIST_NODES"})
    return jsonify({"status": "ok"})

@app.route('/api/send', methods=['POST'])
def send_cmd():
    """
    Primeste un JSON de la frontend si il forwardeaza direct catre gateway.
    Frontend-ul trimite deja formatul corect { cmd, addr, val/slot/target/... }
    """
    payload = request.json
    send_to_mqtt_json(payload)
    return jsonify({"status": "sent"})

@app.route('/api/automations', methods=['GET', 'POST'])
def handle_automations():
    if request.method == 'POST':
        automations.append(request.json)
        save_automations()
        return jsonify({"status": "ok"})
    return jsonify(automations)

@app.route('/api/automations/<int:index>', methods=['PUT', 'DELETE'])
def update_delete_automation(index):
    if 0 <= index < len(automations):
        if request.method == 'PUT':
            automations[index] = request.json
            save_automations()
            return jsonify({"status": "updated"})
        elif request.method == 'DELETE':
            automations.pop(index)
            save_automations()
            return jsonify({"status": "deleted"})
    return jsonify({"error": "invalid index"}), 400

@app.route('/api/automations/<int:index>/toggle', methods=['PATCH'])
def toggle_automation(index):
    if 0 <= index < len(automations):
        automations[index]['enabled'] = not automations[index].get('enabled', True)
        save_automations()
        return jsonify({"status": "toggled", "new_state": automations[index]['enabled']})
    return jsonify({"error": "invalid index"}), 400

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)
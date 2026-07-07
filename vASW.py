
import os
import sys

DEDICATED_HOST_MODE = any(arg.lower() in ("--dedicated-host", "--host-server", "--server") for arg in sys.argv[1:])
if DEDICATED_HOST_MODE:
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

import pygame
import math 
import random
import numpy as np
from scipy.signal import spectrogram
from scipy.signal import butter, filtfilt
import matplotlib.cm as cm
import matplotlib.pyplot as plt
from threading import Timer, Thread, Lock
import time
import pygame_gui
from SimConnect import *
import pyscroll
import pyscroll.data
from pyscroll.group import PyscrollGroup
from pytmx.util_pygame import load_pygame
import json
import traceback
import datetime
import pyaudio
from queue import Queue
import re
import ctypes
import socket
import struct
import uuid
import urllib.request
import urllib.parse
import tempfile
import zipfile
import subprocess

from SimConnect.Constants import SIMCONNECT_UNUSED
from SimConnect.Enum import (
    SIMCONNECT_DATA_INITPOSITION,
    SIMCONNECT_DATA_WAYPOINT,
    SIMCONNECT_DATATYPE,
    SIMCONNECT_WAYPOINT_FLAGS
)


audio_queue = Queue()
lat = None
long = None
alt = None
plane_altitude_ft = None
aircraft_groundspeed_kt = 0.0
xplane = 1
hdg = 120

# ---------------------------------------------------------------------------
# Crash logging
# ---------------------------------------------------------------------------
# pygame apps can otherwise disappear with very little context. Keep a simple
# append-only crash log so runtime exceptions can be inspected after exit.
LOG_FILE = "crash.log"
COASTLINE_FILE = os.path.join("assets", "coastline.geojson")
LAND_FILE = os.path.join("assets", "land.geojson")
GAIST_MODEL_FILE = "gaist_models.json"
GAIST_BOATS_DIR = os.path.join("gaist-ultra-V7_1", "SimObjects", "Boats")
MODEL_LIBRARY_AUTO = "Auto"
MODEL_LIBRARY_MILTECH = "Miltech Mission Hub"
MODEL_LIBRARY_SEAFRONT = "Seafront Sightseeing"
MODEL_LIBRARY_ALL = "All Libraries"
MODEL_LIBRARY_OPTIONS = [
    MODEL_LIBRARY_AUTO,
    MODEL_LIBRARY_MILTECH,
    MODEL_LIBRARY_SEAFRONT,
    MODEL_LIBRARY_ALL
]
MODEL_LIBRARY_DIRS = {
    MODEL_LIBRARY_MILTECH: ["Boats", GAIST_BOATS_DIR],
    MODEL_LIBRARY_SEAFRONT: [
        "seafront-sightseeing-vessels-uksw",
        "seafront-sightseeing-vessels-uk-southeast-v2-6-0",
        "seafront-sightseeing-vessels-core"
    ]
}
TERRAIN_INDEX_CELL_DEG = 2
LAND_RASTER_SIZE = (720, 360)
coastline_polylines = []
land_polygons = []
coastline_index = {}
land_index = {}
land_raster_surface = None
radar_terrain_cache = {
    "key": None,
    "surface": None
}
ship_injection_enabled = False
ship_injection_last_update = 0
ship_injection_update_interval = 0.25
ship_waypoint_update_interval = 0.0
ship_injection_resync_interval = 20.0
ship_motion_resend_interval = 5.0
ship_motion_heading_epsilon_deg = 0.25
ship_motion_speed_epsilon_kts = 0.1
ship_injections = {}
ship_visual_speed_multiplier = 1
ship_visual_lead_seconds = 0.0
ship_deck_hold_radius_nm = 0.04
ship_deck_hold_max_alt_ft = 120.0
ship_deck_speed_tolerance_kt = 3.0
ship_deck_altitude_stable_tolerance_ft = 2.5
ship_deck_lock_settle_seconds = 0.35
ship_deck_release_climb_ft = 4.0
ship_deck_release_climb_rate_fps = 1.5
ship_deck_hard_snap_interval_sec = 5.0
manual_deck_snap_interval_sec = 0.5
manual_deck_snap_deadband_m = 0.75
ship_spawn_aft_forward_nm = -0.06
ship_spawn_aft_right_nm = 0.0
ship_route_arrival_radius_nm = 0.05
ship_deck_carry_enabled = False
manual_deck_lock_track = None
ship_command_accel_kts_s = 0.35
ship_command_decel_kts_s = 0.55
ship_command_turn_rate_dps = 1.5
ship_default_underway_speed_kts = 8.0
shadow_catchup_speed_margin_kts = 8.0
AISHUB_API_URL = "https://data.aishub.net/ws.php"
AISHUB_MIN_REFRESH_SECONDS = 60.0
AISHUB_DEFAULT_MAX_CONTACTS = 1000
CIVILIAN_TRAFFIC_MIN_OFFSHORE_NM = 10.0
CIVILIAN_TRAFFIC_MIN_RANGE_NM = 8.0
CIVILIAN_TRAFFIC_MAX_RANGE_NM = 45.0
CIVILIAN_TRAFFIC_GLOBAL_LAT_LIMIT = 72.0
HOST_CIVILIAN_TRAFFIC_RADIUS_NM = 100.0
HOST_CIVILIAN_TRAFFIC_TARGET_PER_PLAYER = 8
HOST_CIVILIAN_TRAFFIC_REFRESH_SECONDS = 45.0
host_civilian_traffic_last_update = 0.0
aishub_last_fetch_at = 0.0
aishub_last_error = ""
aishub_disabled_notice_shown = False
XPLANE_SHIP_EXPORT_FILE = "xplane_ships.json"
XPLANE_UDP_HOST = "127.0.0.1"
XPLANE_UDP_PORT = 49000
XPLANE_MULTIPLAYER_SHIP_SLOTS = 19
xplane_ship_last_export = 0.0
xplane_ship_export_interval = 0.0
xplane_ship_udp_socket = None
xplane_ship_warning_shown = False
ship_position_definition = None
ship_waypoint_definition = None
ship_motion_definition = None
msfs_splash_effects = []
msfs_splash_warning_shown = False
gaist_model_config = {}
gaist_civilian_model_titles = None
model_library_title_cache = {}
DIFAR_REFERENCE_RANGE_NM = 1.8
DIFAR_MIN_SNR_DB = 6.0
DIFAR_CLEAR_SNR_DB = 35.0
DIFAR_BEARING_RESOLUTION_DEG = 8.0
DIFAR_BEARING_DISPLAY_LENGTH_NM = 8.0
DIFAR_LOCALIZATION_MAX_UNCERT_DEG = 75.0
DIFAR_MAX_DETECTIONS_PER_BUOY = 6
DIFAR_DETECTION_UPDATE_INTERVAL_SEC = 0.20
DIFAR_HIDDEN_DETECTION_UPDATE_INTERVAL_SEC = 1.50
DIFAR_DETECTION_UPDATE_JITTER_SEC = 0.45
SOUND_SPEED_MPS = 1500.0
APP_VERSION = "0.3.3"
GITHUB_OWNER = "DatDerpyWasTaken"
GITHUB_REPO = "vASW"
GITHUB_RELEASE_API = f"https://api.github.com/repos/{GITHUB_OWNER}/{GITHUB_REPO}/releases/latest"
UPDATE_ASSET_KEYWORDS = ("windows", "win", "vASW")

MULTIPLAYER_PORT = 51423
MULTIPLAYER_BROADCAST_INTERVAL = 0.25
MULTIPLAYER_STATE_BROADCAST_INTERVAL = 1.0
MULTIPLAYER_STALE_SECONDS = 8.0
MULTIPLAYER_CONTACT_PASSWORD = os.environ.get("VASW_CONTACT_PASSWORD", "d0yl3!))")
MULTIPLAYER_CONTACT_CONTRIBUTION_INTERVAL = 1.5
SERVER_CONTACT_AUTOSAVE_SECONDS = 300.0
MULTIPLAYER_ID = str(uuid.uuid4())
MULTIPLAYER_CALLSIGN = os.environ.get("COMPUTERNAME", "AIRCRAFT")[:12]
MULTIPLAYER_AIRCRAFT_TYPE = "P8"
MULTIPLAYER_AIRCRAFT_TYPES = ["P8", "P3", "C130", "C30J", "A400", "B350", "EH10", "H60", "SH60", "NH90", "AW139"]
MULTIPLAYER_PLAYER_TYPES = ["Aircraft", "Ship", "Submarine"]
MULTIPLAYER_TEAMS = ["BLUFOR", "REDFOR", "Neutral"]
CONTACT_TEAM_OPTIONS = ["BLUFOR", "REDFOR", "Neutral"]
MULTIPLAYER_PLAYER_TYPE = "Aircraft"
MULTIPLAYER_TEAM = "BLUFOR"
ownship_commanded_heading = 120.0
ownship_current_heading = 120.0
ownship_last_heading = 120.0
ownship_heading_rate_dps = 0.0
ownship_commanded_speed = 0.0
ownship_current_speed = 0.0
ownship_commanded_depth = 0.0
ownship_current_depth = 0.0
ownship_route_waypoints = []
ownship_route_index = 0
ownship_route_active = False
ownship_route_status = "No route"
ownship_control_contact_key = "Auto"
ownship_control_contact_track = None
ownship_control_contact_options = ["Auto"]
ownship_control_initialized_track = None
multiplayer_role = "OFF"
multiplayer_enabled = False
multiplayer_socket = None
multiplayer_last_broadcast = 0.0
multiplayer_last_host_broadcast = 0.0
multiplayer_last_state_broadcast = 0.0
multiplayer_last_contact_contribution = 0.0
multiplayer_host_seen = None
multiplayer_peers = {}
multiplayer_channel_assignments = {}
multiplayer_contact_password = ""
server_contact_autosave_last = 0.0
server_command_queue = Queue()
server_command_thread = None
duplicate_contact_pending_row = None
player_channel_range = (1, 99)

if not os.path.exists(LOG_FILE):
    with open(LOG_FILE, "w") as f:
        f.write("=== Crash Log Initialized ===\n")

def log_crash(exc_type, exc_value, exc_traceback):
    # Skip Ctrl+C interruptions
    if issubclass(exc_type, KeyboardInterrupt):
        sys.__excepthook__(exc_type, exc_value, exc_traceback)
        return

    # Write the crash details
    with open(LOG_FILE, "a") as f:
        f.write("\n=== Crash Detected ===\n")
        f.write(f"Time: {datetime.datetime.now()}\n")
        f.write("".join(traceback.format_exception(exc_type, exc_value, exc_traceback)))
        f.write("\n")

    print("Crash logged to crash.log")

sys.excepthook = log_crash


def load_gaist_model_config(path=GAIST_MODEL_FILE):
    if not os.path.exists(path):
        return {"default": ""}

    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def normalize_model_library(library):
    library = str(library or MODEL_LIBRARY_AUTO).strip()
    return library if library in MODEL_LIBRARY_OPTIONS else MODEL_LIBRARY_AUTO


def discover_model_titles_from_dirs(paths):
    titles = []
    title_re = re.compile(r"^\s*title\s*=\s*(.+?)\s*$", re.IGNORECASE)
    for base_dir in paths:
        if not os.path.isdir(base_dir):
            continue
        for root, _, files in os.walk(base_dir):
            cfg_name = next((name for name in files if name.lower() == "sim.cfg"), None)
            if cfg_name is None:
                continue
            cfg_path = os.path.join(root, cfg_name)
            try:
                with open(cfg_path, "r", encoding="utf-8", errors="ignore") as cfg:
                    text = cfg.read()
            except OSError:
                continue
            for line in text.splitlines():
                match = title_re.match(line)
                if match:
                    titles.append(match.group(1).strip())
    return sorted(set(title for title in titles if title))


def discovered_model_titles_for_library(library):
    library = normalize_model_library(library)
    if library == MODEL_LIBRARY_AUTO:
        library = MODEL_LIBRARY_MILTECH
    if library == MODEL_LIBRARY_ALL:
        titles = []
        for option in (MODEL_LIBRARY_MILTECH, MODEL_LIBRARY_SEAFRONT):
            for title in discovered_model_titles_for_library(option):
                if title not in titles:
                    titles.append(title)
        return sorted(titles)
    if library in model_library_title_cache:
        return model_library_title_cache[library]
    titles = discover_model_titles_from_dirs(MODEL_LIBRARY_DIRS.get(library, []))
    model_library_title_cache[library] = titles
    return titles


def discover_gaist_civilian_model_titles():
    """Find locally installed GAIST boat titles that look civilian."""
    titles = []
    if not os.path.isdir(GAIST_BOATS_DIR):
        return titles

    excluded_keywords = (
        "carrier", "cvn", "destroyer", "frigate", "navy", "naval", "warship",
        "military", "coast guard", "uscg", "hms", "uss", "fns", "ins", "rfa",
        "patrol", "missile", "submarine", "replenishment", "auxiliary",
        "corvette", "landing ship", "lhd", "lpd", "mistral"
    )

    title_re = re.compile(r"^\s*title\s*=\s*(.+?)\s*$", re.IGNORECASE)
    for root, _, files in os.walk(GAIST_BOATS_DIR):
        if "sim.cfg" not in files:
            continue
        cfg_path = os.path.join(root, "sim.cfg")
        try:
            with open(cfg_path, "r", encoding="utf-8", errors="ignore") as cfg:
                text = cfg.read()
        except OSError:
            continue

        haystack = f"{root}\n{text}".lower()
        if any(keyword in haystack for keyword in excluded_keywords):
            continue

        for line in text.splitlines():
            match = title_re.match(line)
            if match:
                titles.append(match.group(1).strip())

    return sorted(set(titles))


def random_gaist_civilian_model_title(model_library=MODEL_LIBRARY_AUTO):
    global gaist_civilian_model_titles

    model_library = normalize_model_library(model_library)
    if model_library not in (MODEL_LIBRARY_AUTO, MODEL_LIBRARY_ALL):
        library_titles = discovered_model_titles_for_library(model_library)
        return random.choice(library_titles) if library_titles else ""

    configured_titles = gaist_model_config.get("civilian_models", [])
    if isinstance(configured_titles, str):
        configured_titles = [configured_titles]
    configured_titles = [
        title for title in configured_titles
        if title and not title.startswith("REPLACE_WITH_")
    ]
    if configured_titles:
        return random.choice(configured_titles)

    if gaist_civilian_model_titles is None:
        gaist_civilian_model_titles = discover_gaist_civilian_model_titles()

    if gaist_civilian_model_titles:
        return random.choice(gaist_civilian_model_titles)

    return gaist_model_config.get("default", "")


def configured_model_title_for_type(contact_type, contact_class):
    model_map = gaist_model_config.get(contact_type, {})
    if isinstance(model_map, str):
        return model_map
    if not isinstance(model_map, dict):
        return ""
    return model_map.get(contact_class) or model_map.get("Unknown") or ""



def all_configured_model_titles():
    titles = []
    for key, value in gaist_model_config.items():
        if key == "splash":
            continue
        if isinstance(value, str):
            titles.append(value)
        elif isinstance(value, list):
            titles.extend(value)
        elif isinstance(value, dict):
            titles.extend(value.values())
    titles.extend(discover_gaist_civilian_model_titles())
    titles.extend(discovered_model_titles_for_library(MODEL_LIBRARY_MILTECH))
    cleaned = []
    for title in titles:
        title = str(title or "").strip()
        if title and not title.startswith("REPLACE_WITH_") and title not in cleaned:
            cleaned.append(title)
    return sorted(cleaned)


def model_titles_for_library(model_library):
    model_library = normalize_model_library(model_library)
    if model_library == MODEL_LIBRARY_AUTO:
        return all_configured_model_titles()
    if model_library == MODEL_LIBRARY_ALL:
        titles = all_configured_model_titles()
        for title in discovered_model_titles_for_library(MODEL_LIBRARY_SEAFRONT):
            if title not in titles:
                titles.append(title)
        return sorted(titles)
    if model_library == MODEL_LIBRARY_MILTECH:
        return all_configured_model_titles()
    return discovered_model_titles_for_library(model_library)


def compact_model_options(selected_model="Auto"):
    selected_model = str(selected_model or "Auto")
    options = ["Auto"]
    if selected_model != "Auto" and selected_model not in options:
        options.append(selected_model)
    return options


def model_options_for_contact(internal_type, internal_class, model_library=MODEL_LIBRARY_AUTO):
    model_library = normalize_model_library(model_library)
    options = ["Auto"]
    if model_library != MODEL_LIBRARY_SEAFRONT:
        preferred = configured_model_title_for_type(internal_type, internal_class)
        if preferred and preferred not in options:
            options.append(preferred)
        if internal_type == "Surface-Ship" and internal_class == "Civilian":
            civilian_titles = gaist_model_config.get("civilian_models", [])
            if isinstance(civilian_titles, str):
                civilian_titles = [civilian_titles]
            for title in civilian_titles:
                if title and title not in options:
                    options.append(title)
    for title in model_titles_for_library(model_library):
        if title not in options:
            options.append(title)
    return options

def resolve_gaist_model_title(model_title):
    model_title = str(model_title or "").strip()
    if not model_title or model_title == "Auto" or model_title.startswith("REPLACE_WITH_"):
        return ""
    missionhub_map = gaist_model_config.get("miltech_missionhub_models", {})
    if isinstance(missionhub_map, dict):
        return missionhub_map.get(model_title, model_title)
    return model_title

def gaist_model_title_for_contact(contact):
    explicit_model = resolve_gaist_model_title(getattr(contact, "gaist_model_title", ""))
    if explicit_model:
        return explicit_model
    contact_type = getattr(contact, "internal_type", "")
    contact_class = getattr(contact, "internal_class", "Unknown")
    model_library = normalize_model_library(getattr(contact, "model_library", MODEL_LIBRARY_AUTO))
    if contact_type == "Surface-Ship" and contact_class == "Civilian":
        if not getattr(contact, "gaist_model_title", ""):
            contact.gaist_model_title = random_gaist_civilian_model_title(model_library)
        model_title = contact.gaist_model_title
    else:
        model_title = ""
        if model_library not in (MODEL_LIBRARY_SEAFRONT,):
            model_title = configured_model_title_for_type(contact_type, contact_class)
            if not model_title and contact_type == "Surface-Ship":
                model_title = configured_model_title_for_type("Surface-Ship", "Unknown")
            if not model_title and contact_type == "Surface-Ship":
                model_title = gaist_model_config.get("default", "")
        if not model_title and contact_type == "Surface-Ship":
            library_titles = model_titles_for_library(model_library)
            model_title = random.choice(library_titles) if library_titles else ""

    return resolve_gaist_model_title(model_title)


def is_dicass_ping_contact(contact):
    return (
        getattr(contact, "name", "") == "DICASS" or
        any(tone.label == "FMCW" for tone in getattr(contact, "tones", []))
    )


def msfs_splash_config():
    splash_config = gaist_model_config.get("splash", {})
    if isinstance(splash_config, str):
        return {"model": splash_config, "duration": 2.5}
    return splash_config


def msfs_splash_model_title():
    model_title = msfs_splash_config().get("model", "")
    if model_title.startswith("REPLACE_WITH_"):
        return ""
    return model_title


def ensure_msfs_connection():
    global sm, aq

    if "sm" in globals() and sm is not None:
        return True

    try:
        sm = SimConnect()
        aq = AircraftRequests(sm, _time=2000)
        return True
    except ConnectionError:
        sm = None
        aq = None
        return False


def ship_sim_speed_kts(contact):
    return max(0.0, float(getattr(contact, "speed", 0) or 0) * ship_visual_speed_multiplier)

def ship_approach_value(current, target, max_delta):
    current = float(current or 0.0)
    target = float(target or 0.0)
    max_delta = max(0.0, float(max_delta or 0.0))
    if current < target:
        return min(target, current + max_delta)
    return max(target, current - max_delta)


def ship_approach_bearing(current, target, max_delta):
    current = float(current or 0.0) % 360.0
    target = float(target or 0.0) % 360.0
    error = (target - current + 180.0) % 360.0 - 180.0
    return (current + max(-max_delta, min(max_delta, error))) % 360.0


def ensure_ship_command_state(contact):
    current_speed = max(0.0, float(getattr(contact, "speed", 0.0) or 0.0))
    current_heading = float(getattr(contact, "bearing", 0.0) or 0.0) % 360.0
    if not hasattr(contact, "commanded_speed"):
        contact.commanded_speed = current_speed
    if not hasattr(contact, "commanded_heading"):
        contact.commanded_heading = current_heading
    if not hasattr(contact, "ship_underway_speed"):
        contact.ship_underway_speed = max(current_speed, float(getattr(contact, "original_speed", 0.0) or 0.0), ship_default_underway_speed_kts)
    return contact


def ship_commanded_stopped(contact):
    ensure_ship_command_state(contact)
    return float(getattr(contact, "commanded_speed", 0.0) or 0.0) <= 0.2


def apply_ship_command_dynamics(contact, dt_seconds):
    if getattr(contact, "internal_type", "") != "Surface-Ship":
        return
    ensure_ship_command_state(contact)
    dt_seconds = max(0.0, float(dt_seconds or 0.0))
    current_speed = max(0.0, float(getattr(contact, "speed", 0.0) or 0.0))
    target_speed = max(0.0, float(getattr(contact, "commanded_speed", current_speed) or 0.0))
    speed_rate = ship_command_accel_kts_s if target_speed > current_speed else ship_command_decel_kts_s
    contact.speed = ship_approach_value(current_speed, target_speed, speed_rate * dt_seconds)
    contact.bearing = ship_approach_bearing(
        float(getattr(contact, "bearing", 0.0) or 0.0),
        float(getattr(contact, "commanded_heading", getattr(contact, "bearing", 0.0)) or 0.0),
        ship_command_turn_rate_dps * dt_seconds
    )

def ship_visual_latlon(contact, lead_seconds=0.0):
    start_lat = float(contact.contact_lat)
    start_lon = float(contact.contact_long)
    try:
        lead_seconds = max(0.0, float(lead_seconds or 0.0))
    except (TypeError, ValueError):
        lead_seconds = 0.0
    if lead_seconds <= 0:
        return start_lat, start_lon
    bearing_deg = float(getattr(contact, "bearing", 0) or 0) % 360
    distance_nm = ship_sim_speed_kts(contact) * lead_seconds / 3600.0
    return destination_from_bearing(start_lat, start_lon, bearing_deg, distance_nm)


def ship_init_position(contact, lead_seconds=0.0, visual_state=None):
    init_pos = SIMCONNECT_DATA_INITPOSITION()
    if visual_state is None:
        visual_lat, visual_lon = ship_visual_latlon(contact, lead_seconds)
        visual_speed = ship_sim_speed_kts(contact)
        visual_bearing = float(getattr(contact, "bearing", 0))
    else:
        visual_lat = float(visual_state.get("lat", contact.contact_lat))
        visual_lon = float(visual_state.get("lon", contact.contact_long))
        visual_speed = float(visual_state.get("speed", ship_sim_speed_kts(contact)))
        visual_bearing = float(visual_state.get("bearing", getattr(contact, "bearing", 0)))
    init_pos.Altitude = 0
    init_pos.Latitude = visual_lat
    init_pos.Longitude = visual_lon
    init_pos.Pitch = 0
    init_pos.Bank = 0
    init_pos.Heading = visual_bearing
    init_pos.OnGround = 1
    init_pos.Airspeed = 0
    return init_pos


def current_aircraft_deck_altitude_ft():
    try:
        return float(plane_altitude_ft if plane_altitude_ft is not None else alt)
    except (TypeError, ValueError):
        return None


def ship_deck_relative_offset_nm(ship_lat, ship_lon, ship_heading_deg, own_lat, own_lon):
    distance_nm = haversine(ship_lat, ship_lon, own_lat, own_lon)
    bearing_to_aircraft = haversine_bearing(ship_lat, ship_lon, own_lat, own_lon)
    rel_rad = math.radians((bearing_to_aircraft - ship_heading_deg + 180.0) % 360.0 - 180.0)
    return math.cos(rel_rad) * distance_nm, math.sin(rel_rad) * distance_nm


def ship_deck_target_latlon(ship_lat, ship_lon, ship_heading_deg, forward_nm, right_nm):
    distance_nm = math.hypot(forward_nm, right_nm)
    if distance_nm <= 1e-7:
        return ship_lat, ship_lon
    rel_bearing_deg = math.degrees(math.atan2(right_nm, forward_nm))
    return destination_from_bearing(ship_lat, ship_lon, (ship_heading_deg + rel_bearing_deg) % 360.0, distance_nm)


def ship_deck_lock_is_active_for_contact(contact):
    if not ship_deck_carry_enabled:
        return False
    injection = ship_injections.get(getattr(contact, "track_number", None))
    return bool(getattr(contact, "manual_deck_lock_active", False) or (injection and injection.get("deck_lock_active")))


def release_ship_deck_lock(injection, reason="released"):
    if injection.get("deck_lock_active"):
        print(f"MSFS deck carry {reason} for track {injection.get('track_number', '?')}")
    injection["deck_lock_active"] = False
    injection["deck_candidate_since"] = None
    injection.pop("deck_lock_forward_nm", None)
    injection.pop("deck_lock_right_nm", None)
    injection.pop("deck_lock_alt_ft", None)
    injection.pop("deck_next_hard_snap_at", None)

def find_contact_by_track_number(track_number):
    for contact in contacts:
        if getattr(contact, "track_number", None) == track_number:
            return contact
    return None


def ship_deck_reference_state(contact):
    injection = ship_injections.get(getattr(contact, "track_number", None))
    if injection is not None:
        ship_lat = float(injection.get("visual_lat", getattr(contact, "contact_lat", 0.0)))
        ship_lon = float(injection.get("visual_lon", getattr(contact, "contact_long", 0.0)))
        ship_heading = float(injection.get("visual_bearing", getattr(contact, "bearing", 0.0)) or 0.0) % 360.0
        ship_speed = float(injection.get("visual_speed", getattr(contact, "speed", 0.0)) or 0.0)
    else:
        ship_lat = float(getattr(contact, "contact_lat", 0.0))
        ship_lon = float(getattr(contact, "contact_long", 0.0))
        ship_heading = float(getattr(contact, "bearing", 0.0) or 0.0) % 360.0
        ship_speed = float(getattr(contact, "speed", 0.0) or 0.0)
    return ship_lat, ship_lon, ship_heading, ship_speed


def clear_contact_deck_lock(contact):
    global manual_deck_lock_track
    contact.manual_deck_lock_active = False
    for attr in ("deck_lock_forward_nm", "deck_lock_right_nm", "deck_lock_alt_ft", "deck_next_hard_snap_at", "deck_last_alt_ft", "deck_last_alt_time"):
        if hasattr(contact, attr):
            delattr(contact, attr)
    injection = ship_injections.get(getattr(contact, "track_number", None))
    if injection is not None:
        release_ship_deck_lock(injection, "manually unlocked")
    if manual_deck_lock_track == getattr(contact, "track_number", None):
        manual_deck_lock_track = None


def set_aircraft_deck_position(altitude_ft, target_lat, target_lon, speed_kts, heading_deg, track_number="?"):
    attempts = (
        (1, "ground"),
        (0, "airborne"),
    )
    last_error = None
    for on_ground, mode_label in attempts:
        try:
            carried = sm.set_pos(
                altitude_ft,
                target_lat,
                target_lon,
                int(round(max(0.0, float(speed_kts or 0.0)))),
                _Heading=float(heading_deg or 0.0),
                _OnGround=on_ground
            )
        except Exception as exc:
            last_error = exc
            carried = False
        if carried:
            return True, mode_label
    if last_error is not None:
        print(f"MSFS deck hard snap failed for track {track_number}: {last_error}")
    else:
        print(
            f"MSFS deck hard snap rejected for track {track_number}: "
            f"{target_lat:.6f}, {target_lon:.6f}, alt {altitude_ft:.1f} ft"
        )
    return False, None


def snap_aircraft_to_contact_deck(contact, force=False):
    global lat, long, manual_deck_lock_track
    if xplane != 0:
        print("Deck lock is only available in MSFS mode")
        return False
    if not ensure_msfs_connection():
        print("Deck lock unavailable: simulator connection unavailable")
        return False
    altitude_ft = current_aircraft_deck_altitude_ft()
    if altitude_ft is None or not math.isfinite(altitude_ft):
        print(f"Deck lock unavailable for track {getattr(contact, 'track_number', '?')}: invalid altitude")
        return False

    now = time.time()
    last_alt = getattr(contact, "deck_last_alt_ft", None)
    last_time = getattr(contact, "deck_last_alt_time", now)
    altitude_delta = 0.0 if last_alt is None else altitude_ft - float(last_alt)
    elapsed = max(0.001, now - float(last_time or now))
    altitude_rate_fps = altitude_delta / elapsed if last_alt is not None else 0.0
    contact.deck_last_alt_ft = altitude_ft
    contact.deck_last_alt_time = now

    lock_alt = float(getattr(contact, "deck_lock_alt_ft", altitude_ft))
    if not force and (altitude_ft > lock_alt + ship_deck_release_climb_ft or altitude_rate_fps > ship_deck_release_climb_rate_fps):
        clear_contact_deck_lock(contact)
        print(f"MSFS deck lock released: climb detected for track {getattr(contact, 'track_number', '?')}")
        return False

    if not force and now < float(getattr(contact, "deck_next_hard_snap_at", 0.0) or 0.0):
        return False

    ship_lat, ship_lon, ship_heading, ship_speed = ship_deck_reference_state(contact)
    target_lat, target_lon = ship_deck_target_latlon(
        ship_lat,
        ship_lon,
        ship_heading,
        float(getattr(contact, "deck_lock_forward_nm", 0.0)),
        float(getattr(contact, "deck_lock_right_nm", 0.0))
    )
    manual_lock_active = getattr(contact, "manual_deck_lock_active", False)
    snap_interval = manual_deck_snap_interval_sec if manual_lock_active else ship_deck_hard_snap_interval_sec
    if manual_lock_active and not force:
        try:
            drift_m = haversine(float(lat), float(long), target_lat, target_lon) * 1852.0
        except (TypeError, ValueError):
            drift_m = manual_deck_snap_deadband_m + 1.0
        if drift_m < manual_deck_snap_deadband_m:
            contact.deck_next_hard_snap_at = now + snap_interval
            return False

    carried, snap_mode = set_aircraft_deck_position(
        altitude_ft,
        target_lat,
        target_lon,
        ship_speed,
        float(hdg or ship_heading or 0.0),
        getattr(contact, "track_number", "?")
    )
    if carried:
        lat = target_lat
        long = target_lon
        snap_interval = manual_deck_snap_interval_sec if getattr(contact, "manual_deck_lock_active", False) else ship_deck_hard_snap_interval_sec
        contact.deck_next_hard_snap_at = now + snap_interval
        manual_deck_lock_track = getattr(contact, "track_number", None)
        if force:
            print(f"MSFS deck hard snap for track {getattr(contact, 'track_number', '?')} ({snap_mode})")
    else:
        contact.deck_next_hard_snap_at = now + 1.0
    return bool(carried)


def force_ship_deck_lock(contact):
    global manual_deck_lock_track
    if not ship_deck_carry_enabled:
        print("Deck lock is disabled")
        return False
    if xplane != 0:
        print("Deck lock is only available in MSFS mode")
        return False
    try:
        own_lat = float(lat)
        own_lon = float(long)
        ship_lat, ship_lon, ship_heading, _ = ship_deck_reference_state(contact)
    except (TypeError, ValueError):
        print(f"Deck lock unavailable for track {getattr(contact, 'track_number', '?')}: invalid position data")
        return False
    altitude_ft = current_aircraft_deck_altitude_ft()
    if altitude_ft is None or not math.isfinite(altitude_ft):
        print(f"Deck lock unavailable for track {getattr(contact, 'track_number', '?')}: invalid altitude")
        return False
    forward_nm, right_nm = ship_deck_relative_offset_nm(ship_lat, ship_lon, ship_heading, own_lat, own_lon)
    contact.manual_deck_lock_active = True
    contact.deck_lock_forward_nm = forward_nm
    contact.deck_lock_right_nm = right_nm
    contact.deck_lock_alt_ft = altitude_ft
    contact.deck_last_alt_ft = altitude_ft
    contact.deck_last_alt_time = time.time()
    contact.deck_next_hard_snap_at = 0.0
    manual_deck_lock_track = getattr(contact, "track_number", None)

    injection = ship_injections.get(getattr(contact, "track_number", None))
    if injection is not None:
        release_ship_deck_lock(injection, "manual deck lock moved to contact snap")

    print(
        f"MSFS deck carry manually locked for track {getattr(contact, 'track_number', '?')} "
        f"offset FWD {forward_nm * 6076.12:.0f} ft RIGHT {right_nm * 6076.12:.0f} ft"
    )
    snap_aircraft_to_contact_deck(contact, force=True)
    return True



def spawn_aircraft_aft_of_ship(contact):
    global lat, long, hdg
    if xplane != 0:
        print("Ship aft spawn is only available in MSFS mode")
        return False
    if not ensure_msfs_connection():
        print("Ship aft spawn unavailable: simulator connection unavailable")
        return False
    if contact is None:
        print("Ship aft spawn ignored: select a surface ship contact first")
        return False

    try:
        ship_lat, ship_lon, ship_heading, ship_speed = ship_deck_reference_state(contact)
        target_lat, target_lon = ship_deck_target_latlon(
            ship_lat,
            ship_lon,
            ship_heading,
            ship_spawn_aft_forward_nm,
            ship_spawn_aft_right_nm
        )
    except (TypeError, ValueError):
        print(f"Ship aft spawn unavailable for track {getattr(contact, 'track_number', '?')}: invalid ship position")
        return False

    altitude_ft = current_aircraft_deck_altitude_ft()
    if altitude_ft is None or not math.isfinite(altitude_ft):
        print(f"Ship aft spawn unavailable for track {getattr(contact, 'track_number', '?')}: invalid aircraft altitude")
        return False

    carried, snap_mode = set_aircraft_deck_position(
        altitude_ft,
        target_lat,
        target_lon,
        ship_speed,
        ship_heading,
        getattr(contact, "track_number", "?")
    )
    if carried:
        lat = target_lat
        long = target_lon
        hdg = ship_heading
        print(
            f"MSFS spawned aircraft aft of ship track {getattr(contact, 'track_number', '?')} "
            f"({snap_mode}, {abs(ship_spawn_aft_forward_nm) * 6076.12:.0f} ft aft)"
        )
        return True
    return False
def toggle_ship_deck_lock(contact):
    if ship_deck_lock_is_active_for_contact(contact):
        clear_contact_deck_lock(contact)
        print(f"MSFS deck lock manually unlocked for track {getattr(contact, 'track_number', '?')}")
        return False
    return force_ship_deck_lock(contact)


def update_manual_deck_lock():
    if not ship_deck_carry_enabled:
        return
    if manual_deck_lock_track is None:
        return
    contact = find_contact_by_track_number(manual_deck_lock_track)
    if contact is None or not getattr(contact, "manual_deck_lock_active", False):
        return
    snap_aircraft_to_contact_deck(contact, force=False)


def carry_aircraft_with_ship_delta(injection, old_lat, old_lon, new_lat, new_lon, visual_speed, bearing_deg):
    global lat, long, plane_altitude_ft, aircraft_groundspeed_kt
    if not ship_deck_carry_enabled or xplane != 0:
        return False

    now = time.time()
    try:
        own_lat = float(lat)
        own_lon = float(long)
        ship_lat = float(new_lat)
        ship_lon = float(new_lon)
        visual_speed = max(0.0, float(visual_speed or 0.0))
        bearing_deg = float(bearing_deg or 0.0) % 360.0
        aircraft_speed = max(0.0, float(aircraft_groundspeed_kt or 0.0))
    except (TypeError, ValueError):
        release_ship_deck_lock(injection, "released: invalid aircraft/ship data")
        return False

    altitude_ft = current_aircraft_deck_altitude_ft()
    if altitude_ft is None or not math.isfinite(altitude_ft):
        release_ship_deck_lock(injection, "released: invalid altitude")
        return False

    last_alt = injection.get("deck_last_alt_ft")
    last_time = injection.get("deck_last_alt_time")
    altitude_delta = 0.0 if last_alt is None else altitude_ft - float(last_alt)
    elapsed = max(0.001, now - float(last_time or now))
    altitude_rate_fps = altitude_delta / elapsed if last_alt is not None else 0.0
    injection["deck_last_alt_ft"] = altitude_ft
    injection["deck_last_alt_time"] = now

    if injection.get("deck_lock_active"):
        lock_alt = float(injection.get("deck_lock_alt_ft", altitude_ft))
        if altitude_ft > lock_alt + ship_deck_release_climb_ft or altitude_rate_fps > ship_deck_release_climb_rate_fps:
            release_ship_deck_lock(injection, "released: climb detected")
            return False

        forward_nm = float(injection.get("deck_lock_forward_nm", 0.0))
        right_nm = float(injection.get("deck_lock_right_nm", 0.0))
        target_lat, target_lon = ship_deck_target_latlon(ship_lat, ship_lon, bearing_deg, forward_nm, right_nm)
        hard_snap_due = now >= float(injection.get("deck_next_hard_snap_at", 0.0) or 0.0)
        if not hard_snap_due:
            return False
        carried, snap_mode = set_aircraft_deck_position(
            altitude_ft,
            target_lat,
            target_lon,
            visual_speed,
            float(hdg or bearing_deg or 0.0),
            injection.get("track_number", "?")
        )
        injection["deck_next_hard_snap_at"] = now + (ship_deck_hard_snap_interval_sec if carried else 1.0)
        if carried:
            print(f"MSFS deck hard snap for track {injection.get('track_number', '?')} ({snap_mode})")
            lat = target_lat
            long = target_lon
        return bool(carried)

    distance_nm = haversine(own_lat, own_lon, ship_lat, ship_lon)
    speed_matches = abs(aircraft_speed - visual_speed) <= ship_deck_speed_tolerance_kt
    altitude_stable = last_alt is not None and abs(altitude_delta) <= ship_deck_altitude_stable_tolerance_ft
    in_capture_zone = distance_nm <= ship_deck_hold_radius_nm and altitude_ft <= ship_deck_hold_max_alt_ft

    if not (in_capture_zone and speed_matches and altitude_stable):
        injection["deck_candidate_since"] = None
        return False

    candidate_since = injection.get("deck_candidate_since")
    if candidate_since is None:
        injection["deck_candidate_since"] = now
        return False
    if now - float(candidate_since) < ship_deck_lock_settle_seconds:
        return False

    forward_nm, right_nm = ship_deck_relative_offset_nm(ship_lat, ship_lon, bearing_deg, own_lat, own_lon)
    injection["deck_lock_active"] = True
    injection["deck_lock_forward_nm"] = forward_nm
    injection["deck_lock_right_nm"] = right_nm
    injection["deck_lock_alt_ft"] = altitude_ft
    injection["deck_next_hard_snap_at"] = now + ship_deck_hard_snap_interval_sec
    print(
        f"MSFS deck carry active for track {injection.get('track_number', '?')} "
        f"offset FWD {forward_nm * 6076.12:.0f} ft RIGHT {right_nm * 6076.12:.0f} ft"
    )
    return True


def advance_ship_visual_state(injection, contact, now):
    last_update = float(injection.get("visual_update_at", now) or now)
    elapsed = max(0.0, min(0.25, now - last_update))
    speed_kts = ship_sim_speed_kts(contact)
    bearing_deg = float(getattr(contact, "bearing", 0) or 0) % 360
    lat = float(injection.get("visual_lat", contact.contact_lat))
    lon = float(injection.get("visual_lon", contact.contact_long))

    old_lat, old_lon = lat, lon
    if elapsed > 0:
        lat, lon = destination_from_bearing(lat, lon, bearing_deg, speed_kts * elapsed / 3600.0)
    carry_aircraft_with_ship_delta(injection, old_lat, old_lon, lat, lon, speed_kts, bearing_deg)
    target_lat, target_lon = ship_visual_latlon(contact, ship_visual_lead_seconds)
    injection["visual_drift_nm"] = haversine(lat, lon, target_lat, target_lon)

    injection["visual_lat"] = lat
    injection["visual_lon"] = lon
    injection["visual_speed"] = speed_kts
    injection["visual_bearing"] = bearing_deg
    injection["visual_update_at"] = now
    return {"lat": lat, "lon": lon, "speed": speed_kts, "bearing": bearing_deg}

def ensure_ship_position_definition():
    global ship_position_definition

    if ship_position_definition is not None:
        return ship_position_definition

    ship_position_definition = sm.new_def_id()
    sm.dll.AddToDataDefinition(
        sm.hSimConnect,
        ship_position_definition.value,
        b"Initial Position",
        b"",
        SIMCONNECT_DATATYPE.SIMCONNECT_DATATYPE_INITPOSITION,
        0,
        SIMCONNECT_UNUSED
    )
    return ship_position_definition


def ensure_ship_waypoint_definition():
    global ship_waypoint_definition

    if ship_waypoint_definition is not None:
        return ship_waypoint_definition

    ship_waypoint_definition = sm.new_def_id()
    sm.dll.AddToDataDefinition(
        sm.hSimConnect,
        ship_waypoint_definition.value,
        b"AI WAYPOINT LIST",
        b"number",
        SIMCONNECT_DATATYPE.SIMCONNECT_DATATYPE_WAYPOINT,
        0,
        SIMCONNECT_UNUSED
    )
    return ship_waypoint_definition


def destination_from_bearing(lat_deg, lon_deg, bearing_deg, distance_nm):
    """Return the lat/lon reached by travelling a distance on a true bearing."""
    earth_radius_nm = 3440.065
    lat1 = math.radians(float(lat_deg))
    lon1 = math.radians(float(lon_deg))
    bearing = math.radians(float(bearing_deg))
    angular_distance = float(distance_nm) / earth_radius_nm

    lat2 = math.asin(
        math.sin(lat1) * math.cos(angular_distance) +
        math.cos(lat1) * math.sin(angular_distance) * math.cos(bearing)
    )
    lon2 = lon1 + math.atan2(
        math.sin(bearing) * math.sin(angular_distance) * math.cos(lat1),
        math.cos(angular_distance) - math.sin(lat1) * math.sin(lat2)
    )

    return math.degrees(lat2), ((math.degrees(lon2) + 540) % 360) - 180


def random_point_within_range_nm(lat_deg, lon_deg, range_nm):
    """Random point inside a circle around the requested lat/lon."""
    try:
        radius_nm = max(0.0, float(range_nm or 0.0))
    except (TypeError, ValueError):
        radius_nm = 0.0
    if radius_nm <= 0:
        return float(lat_deg), float(lon_deg)

    # sqrt gives an even distribution across the circle area, not just radius.
    random_distance_nm = radius_nm * math.sqrt(random.random())
    random_bearing_deg = random.uniform(0, 360)
    return destination_from_bearing(lat_deg, lon_deg, random_bearing_deg, random_distance_nm)


def make_ship_waypoint(lat_deg, lon_deg, speed_kts, flags):
    waypoint = SIMCONNECT_DATA_WAYPOINT()
    waypoint.Latitude = float(lat_deg)
    waypoint.Longitude = float(lon_deg)
    waypoint.Altitude = 0
    waypoint.Flags = int(flags)
    waypoint.ktsSpeed = float(max(0.0, speed_kts))
    waypoint.percentThrottle = 0 if waypoint.ktsSpeed <= 0.1 else 60
    return waypoint


def set_injected_ship_waypoints(object_id, contact, visual_state=None):
    if not ensure_msfs_connection():
        return False

    if visual_state is None:
        speed_kts = ship_sim_speed_kts(contact)
        bearing_deg = float(getattr(contact, "bearing", 0)) % 360
        start_lat, start_lon = ship_visual_latlon(contact, ship_visual_lead_seconds)
    else:
        speed_kts = float(visual_state.get("speed", ship_sim_speed_kts(contact)))
        bearing_deg = float(visual_state.get("bearing", getattr(contact, "bearing", 0))) % 360
        start_lat = float(visual_state.get("lat", contact.contact_lat))
        start_lon = float(visual_state.get("lon", contact.contact_long))
    near_lat, near_lon = destination_from_bearing(start_lat, start_lon, bearing_deg, max(0.20, speed_kts / 360.0))
    far_lat, far_lon = destination_from_bearing(start_lat, start_lon, bearing_deg, max(1.20, speed_kts / 45.0))

    flags = (
        SIMCONNECT_WAYPOINT_FLAGS.SIMCONNECT_WAYPOINT_ON_GROUND |
        SIMCONNECT_WAYPOINT_FLAGS.SIMCONNECT_WAYPOINT_SPEED_REQUESTED |
        SIMCONNECT_WAYPOINT_FLAGS.SIMCONNECT_WAYPOINT_THROTTLE_REQUESTED
    )
    waypoint_array_type = SIMCONNECT_DATA_WAYPOINT * 2
    waypoint_array = waypoint_array_type(
        make_ship_waypoint(near_lat, near_lon, speed_kts, flags),
        make_ship_waypoint(far_lat, far_lon, speed_kts, flags)
    )

    definition = ensure_ship_waypoint_definition()
    try:
        result = sm.dll.SetDataOnSimObject(
            sm.hSimConnect,
            definition.value,
            DWORD(int(object_id)),
            0,
            len(waypoint_array),
            ctypes.sizeof(SIMCONNECT_DATA_WAYPOINT),
            ctypes.cast(waypoint_array, ctypes.c_void_p)
        )
        return sm.IsHR(result, 0)
    except Exception as exc:
        print(f"MSFS ship waypoint update failed: {exc}")
        return False



def ensure_ship_motion_definition():
    global ship_motion_definition

    if ship_motion_definition is not None:
        return ship_motion_definition

    ship_motion_definition = sm.new_def_id()
    sm.dll.AddToDataDefinition(
        sm.hSimConnect,
        ship_motion_definition.value,
        b"AI DESIRED SPEED",
        b"Knots",
        SIMCONNECT_DATATYPE.SIMCONNECT_DATATYPE_FLOAT64,
        0,
        SIMCONNECT_UNUSED
    )
    sm.dll.AddToDataDefinition(
        sm.hSimConnect,
        ship_motion_definition.value,
        b"AI DESIRED HEADING",
        b"Degrees",
        SIMCONNECT_DATATYPE.SIMCONNECT_DATATYPE_FLOAT64,
        0,
        SIMCONNECT_UNUSED
    )
    return ship_motion_definition


def set_injected_ship_motion(object_id, contact, visual_state=None):
    if not ensure_msfs_connection():
        return False

    if visual_state is None:
        speed_kts = ship_sim_speed_kts(contact)
        heading_deg = float(getattr(contact, "bearing", 0) or 0) % 360.0
    else:
        speed_kts = float(visual_state.get("speed", ship_sim_speed_kts(contact)) or 0.0)
        heading_deg = float(visual_state.get("bearing", getattr(contact, "bearing", 0)) or 0.0) % 360.0

    data_array = (ctypes.c_double * 2)(float(max(0.0, speed_kts)), float(heading_deg))
    definition = ensure_ship_motion_definition()
    try:
        result = sm.dll.SetDataOnSimObject(
            sm.hSimConnect,
            definition.value,
            DWORD(int(object_id)),
            0,
            0,
            ctypes.sizeof(data_array),
            ctypes.cast(data_array, ctypes.c_void_p)
        )
        return sm.IsHR(result, 0)
    except Exception as exc:
        print(f"MSFS ship motion update failed: {exc}")
        return False


def heading_delta_deg(a, b):
    return abs((float(a or 0.0) - float(b or 0.0) + 180.0) % 360.0 - 180.0)


def should_send_injected_ship_motion(injection, visual_state, now):
    last_motion_update = float(injection.get("last_motion_update", 0.0) or 0.0)
    if last_motion_update <= 0.0:
        return True
    if now - last_motion_update >= ship_motion_resend_interval:
        return True

    speed = float(visual_state.get("speed", 0.0) or 0.0)
    heading = float(visual_state.get("bearing", 0.0) or 0.0) % 360.0
    last_speed = float(injection.get("last_speed", speed) or 0.0)
    last_heading = float(injection.get("last_heading", heading) or 0.0) % 360.0

    return (
        abs(speed - last_speed) >= ship_motion_speed_epsilon_kts or
        heading_delta_deg(heading, last_heading) >= ship_motion_heading_epsilon_deg
    )


def set_injected_ship_position(object_id, contact, visual_state=None):
    if not ensure_msfs_connection():
        return False

    definition = ensure_ship_position_definition()
    init_pos = ship_init_position(contact, ship_visual_lead_seconds, visual_state)
    try:
        result = sm.dll.SetDataOnSimObject(
            sm.hSimConnect,
            definition.value,
            DWORD(int(object_id)),
            0,
            0,
            ctypes.sizeof(init_pos),
            ctypes.pointer(init_pos)
        )
        return sm.IsHR(result, 0)
    except Exception as exc:
        print(f"MSFS ship position update failed: {exc}")
        return False


def spawn_gaist_ship_for_contact(contact):
    if not ensure_msfs_connection():
        print("MSFS ship injection skipped: simulator connection unavailable")
        return False

    model_title = gaist_model_title_for_contact(contact)
    if not model_title:
        print(
            "MSFS ship injection skipped: no GAIST model for "
            f"{getattr(contact, 'internal_type', 'Unknown')} / {getattr(contact, 'internal_class', 'Unknown')}"
        )
        return False

    request_id = sm.new_request_id()
    os.environ["SIMCONNECT_OBJECT_ID"] = ""
    pending_injection = {
        "request_id": request_id.value,
        "object_id": None,
        "track_number": contact.track_number,
        "model_title": model_title,
        "spawn_requested_at": time.time(),
        "last_lat": contact.contact_lat,
        "last_lon": contact.contact_long,
        "last_heading": getattr(contact, "bearing", 0),
        "last_speed": getattr(contact, "speed", 0),
        "last_position_update": 0,
        "last_waypoint_update": 0,
        "failed": False
    }
    ship_injections[contact.track_number] = pending_injection
    try:
        sm.createSimulatedObject(
            model_title,
            float(contact.contact_lat),
            float(contact.contact_long),
            request_id,
            hdg=float(getattr(contact, "bearing", 0)),
            gnd=1,
            alt=0,
            speed=int(round(ship_sim_speed_kts(contact)))
        )
    except Exception as exc:
        pending_injection["failed"] = True
        pending_injection["failure_reason"] = str(exc)
        print(
            "MSFS ship injection disabled for "
            f"track {contact.track_number} ({getattr(contact, 'internal_type', 'Unknown')} / "
            f"{getattr(contact, 'internal_class', 'Unknown')}) using model '{model_title}': {exc}"
        )
        return False

    assigned_object_id = ""
    for _ in range(40):
        assigned_object_id = os.environ.get("SIMCONNECT_OBJECT_ID", "")
        if assigned_object_id:
            break
        time.sleep(0.025)

    if assigned_object_id:
        object_id = int(assigned_object_id)
        pending_injection["object_id"] = object_id
        now = time.time()
        visual_state = advance_ship_visual_state(pending_injection, contact, now)
        if set_injected_ship_motion(object_id, contact, visual_state):
            pending_injection["last_heading"] = visual_state["bearing"]
            pending_injection["last_speed"] = visual_state["speed"]
            pending_injection["last_motion_update"] = now
        return True

    pending_injection["failed"] = True
    pending_injection["failure_reason"] = "no object id returned"
    print(
        "MSFS ship injection disabled for "
        f"track {contact.track_number} ({getattr(contact, 'internal_type', 'Unknown')} / "
        f"{getattr(contact, 'internal_class', 'Unknown')}) using model '{model_title}': "
        "MSFS did not return an object ID. Check that this exact SimObject title is installed."
    )
    return False


def spawn_msfs_splash(lat_deg, lon_deg):
    global msfs_splash_warning_shown

    model_title = msfs_splash_model_title()
    if not model_title:
        if not msfs_splash_warning_shown:
            print("MSFS splash skipped: set gaist_models.json splash.model to a splash-capable SimObject title")
            msfs_splash_warning_shown = True
        return False

    if not ensure_msfs_connection():
        return False

    request_id = sm.new_request_id()
    os.environ["SIMCONNECT_OBJECT_ID"] = ""
    try:
        sm.createSimulatedObject(
            model_title,
            float(lat_deg),
            float(lon_deg),
            request_id,
            hdg=float(hdg or 0),
            gnd=1,
            alt=0,
            speed=0
        )
    except Exception as exc:
        print(f"MSFS splash injection failed for {model_title}: {exc}")
        return False

    assigned_object_id = ""
    for _ in range(20):
        assigned_object_id = os.environ.get("SIMCONNECT_OBJECT_ID", "")
        if assigned_object_id:
            break
        time.sleep(0.025)

    if assigned_object_id:
        duration = float(msfs_splash_config().get("duration", 2.5))
        msfs_splash_effects.append({
            "object_id": int(assigned_object_id),
            "remove_at": time.time() + max(0.5, duration)
        })
        return True

    print(f"MSFS splash injection pending/no object ID for {model_title}")
    return False


def update_msfs_splashes():
    if not msfs_splash_effects:
        return
    if not ensure_msfs_connection():
        return

    now = time.time()
    remaining = []
    for splash in msfs_splash_effects:
        if now < splash["remove_at"]:
            remaining.append(splash)
            continue

        request_id = sm.new_request_id()
        try:
            sm.dll.AIRemoveObject(sm.hSimConnect, DWORD(int(splash["object_id"])), request_id.value)
        except Exception as exc:
            print(f"MSFS splash removal failed for object {splash.get('object_id')}: {exc}")

    msfs_splash_effects[:] = remaining


def remove_injected_ship(track_number):
    if not ensure_msfs_connection():
        return

    injection = ship_injections.pop(track_number, None)
    if not injection or injection.get("object_id") is None:
        return

    request_id = sm.new_request_id()
    try:
        sm.dll.AIRemoveObject(sm.hSimConnect, DWORD(int(injection["object_id"])), request_id.value)
    except Exception as exc:
        print(f"MSFS ship removal failed for track {track_number}: {exc}")


def remove_all_injected_ships():
    for track_number in list(ship_injections.keys()):
        remove_injected_ship(track_number)


def contact_is_msfs_ship_candidate(contact):
    return (
        getattr(contact, "internal_type", "") == "Surface-Ship" and
        bool(gaist_model_title_for_contact(contact)) and
        not is_dicass_ping_contact(contact)
    )


def update_msfs_ship_injections():
    global ship_injection_last_update

    if not ship_injection_enabled:
        return
    now = time.time()
    if ship_injection_update_interval > 0 and now - ship_injection_last_update < ship_injection_update_interval:
        return
    ship_injection_last_update = now

    if not ensure_msfs_connection():
        return

    active_track_numbers = set()
    for contact in contacts:
        if not contact_is_msfs_ship_candidate(contact):
            continue

        active_track_numbers.add(contact.track_number)
        injection = ship_injections.get(contact.track_number)
        if injection is None:
            spawn_gaist_ship_for_contact(contact)
            continue

        current_model_title = gaist_model_title_for_contact(contact)
        if injection.get("failed"):
            if current_model_title != injection.get("model_title"):
                ship_injections.pop(contact.track_number, None)
                spawn_gaist_ship_for_contact(contact)
            continue

        object_id = injection.get("object_id")
        now = time.time()
        if object_id is None:
            assigned_object_id = os.environ.get("SIMCONNECT_OBJECT_ID", "")
            if assigned_object_id:
                object_id = int(assigned_object_id)
                injection["object_id"] = object_id
                visual_state = advance_ship_visual_state(injection, contact, now)
                if set_injected_ship_motion(object_id, contact, visual_state):
                    injection["last_heading"] = visual_state["bearing"]
                    injection["last_speed"] = visual_state["speed"]
                    injection["last_motion_update"] = now
            elif now - injection.get("spawn_requested_at", now) >= 5.0:
                injection["failed"] = True
                injection["failure_reason"] = "no object id returned"
                print(
                    "MSFS ship injection disabled for "
                    f"track {contact.track_number} ({getattr(contact, 'internal_type', 'Unknown')} / "
                    f"{getattr(contact, 'internal_class', 'Unknown')}) using model '{injection.get('model_title', current_model_title)}': "
                    "MSFS did not return an object ID. Check that this exact SimObject title is installed."
                )
                continue

        if object_id is not None:
            visual_state = advance_ship_visual_state(injection, contact, now)
            if should_send_injected_ship_motion(injection, visual_state, now):
                if set_injected_ship_motion(object_id, contact, visual_state):
                    injection["last_heading"] = visual_state["bearing"]
                    injection["last_speed"] = visual_state["speed"]
                    injection["last_motion_update"] = now


    for track_number in list(ship_injections.keys()):
        if track_number not in active_track_numbers:
            remove_injected_ship(track_number)


def contact_is_xplane_ship_candidate(contact):
    return (
        getattr(contact, "internal_type", "") == "Surface-Ship" and
        bool(gaist_model_title_for_contact(contact)) and
        not is_dicass_ping_contact(contact)
    )


def xplane_ship_payload_for_contact(contact):
    return {
        "track": int(getattr(contact, "track_number", 0) or 0),
        "name": str(getattr(contact, "name", "SHIP")),
        "type": str(getattr(contact, "internal_type", "Surface-Ship")),
        "class": str(getattr(contact, "internal_class", "Unknown")),
        "model": gaist_model_title_for_contact(contact),
        "lat": float(getattr(contact, "contact_lat", 0.0) or 0.0),
        "lon": float(getattr(contact, "contact_long", 0.0) or 0.0),
        "alt_m": 0.0,
        "heading": float(getattr(contact, "bearing", 0.0) or 0.0) % 360,
        "speed_kt": float(getattr(contact, "speed", 0.0) or 0.0),
        "broadcasting": bool(getattr(contact, "broadcasting", False))
    }


def xplane_ship_payloads():
    return [
        xplane_ship_payload_for_contact(contact)
        for contact in contacts
        if contact_is_xplane_ship_candidate(contact)
    ]


def write_xplane_ship_export(payloads):
    data = {
        "enabled": bool(ship_injection_enabled and xplane == 1),
        "updated_utc": datetime.datetime.utcnow().isoformat(timespec="milliseconds") + "Z",
        "source": "vASW",
        "ships": payloads
    }
    temp_path = XPLANE_SHIP_EXPORT_FILE + ".tmp"
    with open(temp_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    os.replace(temp_path, XPLANE_SHIP_EXPORT_FILE)


def ensure_xplane_ship_udp_socket():
    global xplane_ship_udp_socket
    if xplane_ship_udp_socket is None:
        xplane_ship_udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return xplane_ship_udp_socket


def send_xplane_dref(dataref, value):
    sock = ensure_xplane_ship_udp_socket()
    encoded_name = dataref.encode("utf-8")[:499]
    packet = b"DREF\0" + struct.pack("<f", float(value)) + encoded_name + (b"\0" * (500 - len(encoded_name)))
    sock.sendto(packet, (XPLANE_UDP_HOST, XPLANE_UDP_PORT))


def update_xplane_multiplayer_ship_slots(payloads):
    # Uses X-Plane's writable multiplayer-position datarefs as a lightweight
    # visual bridge. A proper X-Plane plugin can read xplane_ships.json for
    # actual ship OBJ placement; these slots at least make targets visible.
    for slot in range(1, XPLANE_MULTIPLAYER_SHIP_SLOTS + 1):
        if slot <= len(payloads):
            ship = payloads[slot - 1]
            send_xplane_dref(f"sim/multiplayer/position/plane{slot}_lat", ship["lat"])
            send_xplane_dref(f"sim/multiplayer/position/plane{slot}_lon", ship["lon"])
            send_xplane_dref(f"sim/multiplayer/position/plane{slot}_el", max(0.0, ship.get("alt_m", 0.0)))
            send_xplane_dref(f"sim/multiplayer/position/plane{slot}_psi", ship["heading"])
            send_xplane_dref(f"sim/multiplayer/position/plane{slot}_the", 0.0)
            send_xplane_dref(f"sim/multiplayer/position/plane{slot}_phi", 0.0)
        else:
            # Move unused slots far below sea level so stale targets vanish.
            send_xplane_dref(f"sim/multiplayer/position/plane{slot}_el", -10000.0)


def clear_xplane_ship_export():
    global xplane_ship_last_export
    xplane_ship_last_export = 0.0
    try:
        write_xplane_ship_export([])
        update_xplane_multiplayer_ship_slots([])
    except OSError as exc:
        print(f"X-Plane ship bridge clear failed: {exc}")
    except Exception as exc:
        print(f"X-Plane ship UDP clear failed: {exc}")


def update_xplane_ship_injections():
    global xplane_ship_last_export, xplane_ship_warning_shown
    if not ship_injection_enabled or xplane != 1:
        return
    now = time.time()
    if xplane_ship_export_interval > 0 and now - xplane_ship_last_export < xplane_ship_export_interval:
        return
    xplane_ship_last_export = now

    payloads = xplane_ship_payloads()
    try:
        write_xplane_ship_export(payloads)
    except OSError as exc:
        print(f"X-Plane ship export failed: {exc}")
        return

    try:
        update_xplane_multiplayer_ship_slots(payloads[:XPLANE_MULTIPLAYER_SHIP_SLOTS])
    except Exception as exc:
        if not xplane_ship_warning_shown:
            print(f"X-Plane multiplayer ship UDP failed: {exc}")
            xplane_ship_warning_shown = True


def update_ship_injections():
    if xplane == 1:
        update_xplane_ship_injections()
    else:
        update_msfs_ship_injections()


gaist_model_config = load_gaist_model_config()


# ---------------------------------------------------------------------------
# pygame/audio setup
# ---------------------------------------------------------------------------
pygame.init()
try:
    pygame.scrap.init()
except Exception:
    pass
try:
    pygame.mixer.init()
except Exception:
    if not DEDICATED_HOST_MODE:
        raise


submarine_longitude = None
submarine_latitude = None
submarine_depth = 1040
submarine_speed = random.randint(0,20)
submarine_bearing = random.randint(0,360)
submarine_class = None

fs = 7000
CHUNK = 1024
SPECTROGRAM_UPDATE_INTERVAL_SEC = 0.08
SPECTROGRAM_CHUNK_SECONDS = 0.36
LISTEN_AUDIO_CHUNK_SECONDS = 0.08
current_channel = None
running_audio = not DEDICATED_HOST_MODE
if DEDICATED_HOST_MODE:
    p = None
    stream = None
else:
    p = pyaudio.PyAudio()
    stream = p.open(format=pyaudio.paFloat32,
                channels=1,
                rate=fs,
                output=True,
                frames_per_buffer=CHUNK)
listening_spectrogram_slot = None
manual_azigram_bearing_lines = []
last_listen_audio_time = 0.0
listen_audio_lock = Lock()
listen_audio_thread = None

INTERNAL_WIDTH = 1920
INTERNAL_HEIGHT = 1080
internal_surface = pygame.Surface((INTERNAL_WIDTH, INTERNAL_HEIGHT))
font = pygame.font.SysFont("mono", 12,bold=False)
info = pygame.display.Info()
screen_height = 1080
screen_width = 1920
screen = pygame.display.set_mode((screen_width, screen_height), pygame.RESIZABLE)
display_scale = 1.0
display_viewport_rect = pygame.Rect(0, 0, INTERNAL_WIDTH, INTERNAL_HEIGHT)


def update_display_viewport():
    global display_scale, display_viewport_rect
    display_scale = min(screen_width / INTERNAL_WIDTH, screen_height / INTERNAL_HEIGHT)
    viewport_w = max(1, int(INTERNAL_WIDTH * display_scale))
    viewport_h = max(1, int(INTERNAL_HEIGHT * display_scale))
    display_viewport_rect = pygame.Rect(
        (screen_width - viewport_w) // 2,
        (screen_height - viewport_h) // 2,
        viewport_w,
        viewport_h
    )


update_display_viewport()
try:
    pygame.scrap.init()
except Exception:
    pass
in_menu = True
clock = pygame.time.Clock()
running = True
dt = 0
sonoD_surfacea = pygame.image.load('assets/sonoD.png')
sonoS_surfacea = pygame.image.load('assets/sonoS.png')
sonoB_surface = pygame.image.load('assets/sonoB.png')
sonoT_surfacea = pygame.image.load('assets/aircraft.png')
unknown_contact_surfacea = pygame.image.load('assets/unknown.png')
sub_surfacea = pygame.image.load('assets/submarine.png')
scale_factor = 2
width, height = sonoD_surfacea.get_size()
new_size = (width * scale_factor, height * scale_factor)
sonoT_surface = pygame.transform.scale(sonoT_surfacea, new_size)
sonoS_surface = pygame.transform.scale(sonoS_surfacea, new_size)
sonoD_surface = pygame.transform.scale(sonoD_surfacea, new_size)
unknown_contact_surface = pygame.transform.scale(unknown_contact_surfacea, new_size)
sub_surface = pygame.transform.scale(sub_surfacea, new_size)
menu_surface = pygame.Surface((screen_width,screen_height), pygame.SRCALPHA)




manager = pygame_gui.UIManager((1920, 1080),'menu_theme')
ui_elements = []
BASE_UI_THEME_PATH = "menu_theme"


def scaled_font_size(base_size):
    return max(1, int(round(float(base_size) * max(0.55, display_scale))))


def scaled_sys_font(size, bold=False):
    return pygame.font.SysFont("mono", scaled_font_size(size), bold=bold)


def apply_scaled_ui_theme():
    try:
        with open(BASE_UI_THEME_PATH, "r", encoding="utf-8") as theme_file:
            theme_data = json.load(theme_file)
        for block in theme_data.values():
            font_block = block.get("font") if isinstance(block, dict) else None
            if isinstance(font_block, dict) and "size" in font_block:
                font_block["size"] = scaled_font_size(font_block["size"])
        theme = manager.create_new_theme(theme_data)
        manager.set_ui_theme(theme, update_all_sprites=True)
        manager.rebuild_all_from_changed_theme_data(theme)
    except Exception as exc:
        print(f"UI font scale skipped: {exc}")


def refresh_runtime_fonts():
    global font, contact_menu_font
    font = scaled_sys_font(12)
    if "contact_menu_font" in globals():
        contact_menu_font = scaled_sys_font(17)
    if hasattr(draw_azigram_colour_key, "font"):
        draw_azigram_colour_key.font = scaled_sys_font(9, bold=True)
    if "spectrogram_slot_array" in globals():
        for slot in spectrogram_slot_array:
            slot.marker_font = scaled_sys_font(10, bold=True)
    if "spectro_array" in globals():
        for spectro in spectro_array:
            spectro.tick_font = scaled_sys_font(12)
            spectro.tooltip_font = scaled_sys_font(14)


def refresh_resolution_dependent_fonts():
    apply_scaled_ui_theme()
    refresh_runtime_fonts()
def register_ui_element(element, base_rect):
    ui_elements.append((element, base_rect))


def point_over_visible_ui(screen_pos):
    for element, _ in list(ui_elements):
        try:
            if hasattr(element, "alive") and not element.alive():
                continue
            if hasattr(element, "visible") and not element.visible:
                continue
            if hasattr(element, "rect") and element.rect.collidepoint(screen_pos):
                return True
        except Exception:
            continue
    return False


STATE_MENU = "menu"
STATE_GAME = "game"
state = STATE_MENU

REAL_SUBMARINE_CLASS_OPTIONS = ["Kilo", "Akula", "Delta", "Borei", "Yasen"]

internal_contact_type_list = {
    "Sub-surface": ["Random"] + REAL_SUBMARINE_CLASS_OPTIONS + ["American", "Russian", "Illegal", "Submerged", "Unknown"],
    "Surface-Ship": [
        "Destroyer", "Frigate", "Carrier", "Civilian", "Cruise", "Fishing", "Hospital", "Cutter",
        "Tanker", "Triton", "Adelaide", "Juan Carlos", "Arleigh Burke", "Independence",
        "Ticonderoga", "Sinking Container Ship", "Sinking Sail Ship", "Unknown"
    ],
    "Air": ["Aircraft", "Helicopter", "Unknown"],
    "Land": ["Fixed Site", "Vehicle", "Unknown"],
    "Biological": ["Whale", "Dolphin", "Krill"]
}

# Panel to act as a visible border/frame
contact_panel = pygame_gui.elements.UIPanel(
    relative_rect=pygame.Rect((10, 54), (1900, 560)),  # slightly taller for contact name
    manager=manager     # base layer, uses your panel theme
)

# Scrolling container inside the panel
contact_define_container = pygame_gui.elements.UIScrollingContainer(
    relative_rect=pygame.Rect((0, 0), (1900, 560)),  # fill the panel
    manager=manager,
    container=contact_panel
)


def update_contact_define_scroll_area():
    if not contact_define_row_array:
        contact_define_container.set_scrollable_area_dimensions((1880, 540))
        return

    max_right = 0
    max_bottom = 0
    for row in contact_define_row_array:
        max_right = max(max_right, row.row_panel.relative_rect.right)
        max_bottom = max(max_bottom, row.row_panel.relative_rect.bottom)

    contact_define_container.set_scrollable_area_dimensions((
        max(1880, max_right + 40),
        max(540, max_bottom + 40)
    ))


def scroll_contact_define_container(wheel_steps, mouse_pos=None):
    if not getattr(contact_panel, "visible", True):
        return False

    mouse_pos = mouse_pos or pygame.mouse.get_pos()
    if not contact_panel.rect.collidepoint(mouse_pos):
        return False

    update_contact_define_scroll_area()
    use_vertical = bool(pygame.key.get_mods() & pygame.KMOD_SHIFT)
    primary_bar = contact_define_container.vert_scroll_bar if use_vertical else contact_define_container.horiz_scroll_bar
    fallback_bar = contact_define_container.horiz_scroll_bar if use_vertical else contact_define_container.vert_scroll_bar
    scroll_bar = primary_bar if getattr(primary_bar, "is_enabled", False) else fallback_bar
    if not getattr(scroll_bar, "is_enabled", False):
        return True

    new_start = max(0.0, min(1.0, scroll_bar.start_percentage - (wheel_steps * 0.08)))
    scroll_bar.set_scroll_from_start_percentage(new_start)
    return True


def is_valid_lat(text):
    # Format: optional -, digits, optional decimal
    if not re.fullmatch(r"-?\d+(\.\d+)?", text):
        return False
    
    value = float(text)
    return -90 <= value <= 90


def is_valid_lon(text):
    if not re.fullmatch(r"-?\d+(\.\d+)?", text):
        return False
    
    value = float(text)
    return -180 <= value <= 180


def force_dropdown_down(dropdown, height_limit=220):
    """Keep pygame_gui dropdowns expanding downward after rebuild/recreation."""
    dropdown.expand_direction = "down"
    dropdown.expansion_height_limit = height_limit
    for state in getattr(dropdown, "menu_states", {}).values():
        if hasattr(state, "expand_direction"):
            state.expand_direction = "down"
    return dropdown


def force_dropdown_up(dropdown, height_limit=220):
    """Keep bottom control dropdowns expanding upward and outside compact panels."""
    dropdown.expand_direction = "up"
    dropdown.expansion_height_limit = height_limit
    for state in getattr(dropdown, "menu_states", {}).values():
        if hasattr(state, "expand_direction"):
            state.expand_direction = "up"
    return dropdown


def make_contact_row_dropdown(options, selected, position, container):
    return force_dropdown_down(pygame_gui.elements.UIDropDownMenu(
        options_list=options,
        starting_option=selected,
        relative_rect=pygame.Rect(position, (200, 30)),
        manager=manager,
        container=container,
        expansion_height_limit=220
    ))


class ContactDefineRow:
    def __init__(self, y, manager, container):
        self.y = y

        # State flags
        self.name_entered = False
        self.lat_entered = False
        self.long_entered = False
        self.range_entered = False
        self.internal_type_entered = "Sub-surface"
        self.internal_class_entered = "Akula"
        self.selected_model_library = MODEL_LIBRARY_AUTO
        self.selected_model = "Auto"
        self.class_entered = self.internal_class_entered
        self.speed_entered = False
        self.bearing_entered = False
        self.depth_entered = False
        self.broadcasting_entered = False
        self.team_entered = "Neutral"
        self.route_text_entered = ""
        self.shadow_target_entered = ""
        self.shadow_distance_nm_entered = 5.0
        self.saved = False

        # Layout constants
        LABEL_H = 32   # IMPORTANT: big enough for UITextBox
        INPUT_H = 30
        GAP = 4
        SECTION_GAP = 8
        WIDTH = 200
        X = 10
        X2 = 220

        current_y = 5

        def add_section(label_text, x=X):
            nonlocal current_y

            label = pygame_gui.elements.UITextBox(
                html_text=label_text,
                relative_rect=pygame.Rect((x, current_y), (WIDTH, LABEL_H)),
                manager=manager,
                container=self.row_panel
            )

            input_rect_y = current_y + LABEL_H + GAP

            current_y += LABEL_H + GAP + INPUT_H + SECTION_GAP

            return label, input_rect_y

        # Panel (auto-sized to fit everything)
        self.row_panel = pygame_gui.elements.UIPanel(
            relative_rect=pygame.Rect((5, y), (440, 720)),
            manager=manager,
            container=container
        )

        # Contact Name
        self.contact_name_label, y_pos = add_section("Contact Name")
        self.contact_name_textbox = pygame_gui.elements.UITextEntryLine(
            relative_rect=pygame.Rect((X, y_pos), (WIDTH, INPUT_H)),
            manager=manager,
            container=self.row_panel
        )

        # Latitude
        self.sub_lat_label, y_pos = add_section("Latitude")
        self.sub_lat_textbox = pygame_gui.elements.UITextEntryLine(
            relative_rect=pygame.Rect((X, y_pos), (WIDTH, INPUT_H)),
            manager=manager,
            container=self.row_panel

        )

        # Longitude
        self.sub_long_label, y_pos = add_section("Longitude")
        self.sub_long_textbox = pygame_gui.elements.UITextEntryLine(
            relative_rect=pygame.Rect((X, y_pos), (WIDTH, INPUT_H)),
            manager=manager,
            container=self.row_panel
        )

        # Range
        self.sub_range_label, y_pos = add_section("Range (NM)")
        self.sub_range_textbox = pygame_gui.elements.UITextEntryLine(
            relative_rect=pygame.Rect((X, y_pos), (WIDTH, INPUT_H)),
            manager=manager,
            container=self.row_panel
        )

        # Speed
        self.sub_speed_label, y_pos = add_section("Speed (knots)")
        self.sub_speed_textbox = pygame_gui.elements.UITextEntryLine(
            relative_rect=pygame.Rect((X, y_pos), (WIDTH, INPUT_H)),
            manager=manager,
            container=self.row_panel
        )

        # Bearing
        self.sub_bearing_label, y_pos = add_section("Bearing (°)")
        self.sub_bearing_textbox = pygame_gui.elements.UITextEntryLine(
            relative_rect=pygame.Rect((X, y_pos), (WIDTH, INPUT_H)),
            manager=manager,
            container=self.row_panel
        )

        # Depth
        self.sub_depth_label, y_pos = add_section("Depth (m)")
        self.sub_depth_textbox = pygame_gui.elements.UITextEntryLine(
            relative_rect=pygame.Rect((X, y_pos), (WIDTH, INPUT_H)),
            manager=manager,
            container=self.row_panel
        )

        # Route
        self.route_label, y_pos = add_section("Route")
        self.route_textbox = pygame_gui.elements.UITextEntryLine(
            relative_rect=pygame.Rect((X, y_pos), (WIDTH, INPUT_H)),
            manager=manager,
            container=self.row_panel
        )

        # Shadow target by contact name. Blank means independent movement.
        self.shadow_target_label, y_pos = add_section("Shadow Target")
        self.shadow_target_textbox = pygame_gui.elements.UITextEntryLine(
            relative_rect=pygame.Rect((X, y_pos), (WIDTH, INPUT_H)),
            manager=manager,
            container=self.row_panel
        )

        truth_y = 5
        self.internal_type_label = pygame_gui.elements.UITextBox(
            html_text="Type",
            relative_rect=pygame.Rect((X2, truth_y), (WIDTH, LABEL_H)),
            manager=manager,
            container=self.row_panel
        )
        self.internal_type_dropdown = make_contact_row_dropdown(
            list(internal_contact_type_list.keys()),
            "Sub-surface",
            (X2, truth_y + LABEL_H + GAP),
            self.row_panel
        )

        truth_y += LABEL_H + GAP + INPUT_H + SECTION_GAP
        self.internal_class_label = pygame_gui.elements.UITextBox(
            html_text="Class",
            relative_rect=pygame.Rect((X2, truth_y), (WIDTH, LABEL_H)),
            manager=manager,
            container=self.row_panel
        )
        self.internal_class_dropdown = make_contact_row_dropdown(
            internal_contact_type_list["Sub-surface"],
            "Akula",
            (X2, truth_y + LABEL_H + GAP),
            self.row_panel
        )

        truth_y += LABEL_H + GAP + INPUT_H + SECTION_GAP
        self.model_library_label = pygame_gui.elements.UITextBox(
            html_text="Library",
            relative_rect=pygame.Rect((X2, truth_y), (WIDTH, LABEL_H)),
            manager=manager,
            container=self.row_panel
        )
        self.model_library_dropdown = make_contact_row_dropdown(
            MODEL_LIBRARY_OPTIONS,
            MODEL_LIBRARY_AUTO,
            (X2, truth_y + LABEL_H + GAP),
            self.row_panel
        )

        truth_y += LABEL_H + GAP + INPUT_H + SECTION_GAP
        self.model_label = pygame_gui.elements.UITextBox(
            html_text="Model",
            relative_rect=pygame.Rect((X2, truth_y), (WIDTH, LABEL_H)),
            manager=manager,
            container=self.row_panel
        )
        self.model_dropdown_has_full_options = False
        self.model_dropdown = make_contact_row_dropdown(
            compact_model_options("Auto"),
            "Auto",
            (X2, truth_y + LABEL_H + GAP),
            self.row_panel
        )

        truth_y += LABEL_H + GAP + INPUT_H + SECTION_GAP
        self.team_label = pygame_gui.elements.UITextBox(
            html_text="Team",
            relative_rect=pygame.Rect((X2, truth_y), (WIDTH, LABEL_H)),
            manager=manager,
            container=self.row_panel
        )
        self.team_dropdown = make_contact_row_dropdown(
            CONTACT_TEAM_OPTIONS,
            self.team_entered,
            (X2, truth_y + LABEL_H + GAP),
            self.row_panel
        )

        truth_y += LABEL_H + GAP + INPUT_H + SECTION_GAP
        self.broadcasting_checkbox = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect((X2, truth_y + 4), (160, 30)),
            text="BCAST OFF",
            manager=manager,
            container=self.row_panel
        )
        self.sync_broadcasting_checkbox()

        # Buttons (kept on the right so they stay inside the visible card area)
        self.define_contact_button = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect((X2, truth_y + 44), (96, 40)),
            text="+",
            manager=manager,
            container=self.row_panel
        )
        self.delete_contact_button = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect((X2 + 104, truth_y + 44), (96, 40)),
            text="DEL",
            manager=manager,
            container=self.row_panel
        )

        self.sub_lat_textbox.set_allowed_characters(
            ['0','1','2','3','4','5','6','7','8','9','.','-']
        )
        self.sub_long_textbox.set_allowed_characters(
            ['0','1','2','3','4','5','6','7','8','9','.','-']
        )
        self.sub_range_textbox.set_allowed_characters(
            ['0','1','2','3','4','5','6','7','8','9']
        )
        self.sub_speed_textbox.set_allowed_characters(
            ['0','1','2','3','4','5','6','7','8','9']
        )
        self.sub_bearing_textbox.set_allowed_characters(
            ['0','1','2','3','4','5','6','7','8','9']
        )
        self.sub_depth_textbox.set_allowed_characters(
            ['0','1','2','3','4','5','6','7','8','9']
        )


    def is_route_enabled(self):
        return self.internal_type_entered == "Surface-Ship"


    def update_route_visibility(self):
        self.route_label.show()
        self.route_textbox.show()


    def sync_broadcasting_checkbox(self):
        new_colour = pygame.Color("#99C979") if self.broadcasting_entered else pygame.Color("#b13b3b")
        self.broadcasting_checkbox.set_text("BCAST ON" if self.broadcasting_entered else "BCAST OFF")
        self.broadcasting_checkbox.colours["normal_bg"] = new_colour
        self.broadcasting_checkbox.colours["hovered_bg"] = new_colour
        self.broadcasting_checkbox.colours["active_bg"] = new_colour
        self.broadcasting_checkbox.rebuild()


    def set_broadcasting(self, broadcasting):
        self.broadcasting_entered = bool(broadcasting)
        self.sync_broadcasting_checkbox()


    def set_team(self, team):
        if team not in CONTACT_TEAM_OPTIONS:
            team = "Neutral"
        self.team_entered = team
        self.team_label.set_text('<font color="#99C979">Team</font>')
        self.team_dropdown.kill()
        self.team_dropdown = make_contact_row_dropdown(
            CONTACT_TEAM_OPTIONS,
            team,
            (220, 349),
            self.row_panel
        )
        self.update_route_visibility()

    def refresh_model_library_dropdown(self):
        self.model_library_label.set_text(
            '<font color="#99C979">Library</font>' if self.selected_model_library != MODEL_LIBRARY_AUTO else "Library"
        )
        self.model_library_dropdown.kill()
        self.model_library_dropdown = make_contact_row_dropdown(
            MODEL_LIBRARY_OPTIONS,
            self.selected_model_library,
            (220, 193),
            self.row_panel
        )

    def set_model_library(self, model_library, selected_model=None, full_model_options=True):
        self.selected_model_library = normalize_model_library(model_library)
        self.refresh_model_library_dropdown()
        self.refresh_model_dropdown(self.selected_model if selected_model is None else selected_model, full_options=full_model_options)

    def refresh_model_dropdown(self, selected_model=None, full_options=True):
        selected_model = selected_model if selected_model is not None else self.selected_model
        if full_options:
            options = model_options_for_contact(
                self.internal_type_entered,
                self.internal_class_entered,
                self.selected_model_library
            )
            if selected_model not in options:
                selected_model = "Auto"
        else:
            options = compact_model_options(selected_model)
        self.selected_model = selected_model
        self.model_dropdown_has_full_options = bool(full_options)
        self.model_label.set_text('<font color="#99C979">Model</font>' if selected_model != "Auto" else "Model")
        self.model_dropdown.kill()
        self.model_dropdown = make_contact_row_dropdown(
            options,
            selected_model,
            (220, 271),
            self.row_panel
        )


    def ensure_full_model_dropdown(self):
        if getattr(self, "model_dropdown_has_full_options", False):
            return
        selected_model = self.selected_model
        options = model_options_for_contact(
            self.internal_type_entered,
            self.internal_class_entered,
            self.selected_model_library
        )
        if selected_model not in options:
            options.append(selected_model)
        existing = set(compact_model_options(selected_model))
        additional_options = [option for option in options if option not in existing]
        if additional_options:
            self.model_dropdown.add_options(additional_options)
        self.model_dropdown_has_full_options = True


    def set_model(self, model_title):
        self.refresh_model_dropdown(model_title)


    def set_internal_type(self, internal_type, internal_class=None, selected_model=None, refresh_model=True):
        if selected_model is None:
            selected_model = "Auto"
        if internal_type not in internal_contact_type_list:
            internal_type = "Sub-surface"

        class_options = internal_contact_type_list[internal_type]
        if internal_class not in class_options:
            internal_class = class_options[0]

        self.internal_type_entered = internal_type
        self.internal_class_entered = internal_class
        self.internal_type_label.set_text('<font color="#99C979">Type</font>')
        self.internal_class_label.set_text('<font color="#99C979">Class</font>')

        self.internal_type_dropdown.kill()
        self.internal_type_dropdown = make_contact_row_dropdown(
            list(internal_contact_type_list.keys()),
            internal_type,
            (220, 37),
            self.row_panel
        )

        self.internal_class_dropdown.kill()
        self.internal_class_dropdown = make_contact_row_dropdown(
            class_options,
            internal_class,
            (220, 115),
            self.row_panel
        )

        self.class_entered = internal_class if internal_type == "Sub-surface" else ""
        if refresh_model:
            self.refresh_model_dropdown(selected_model, full_options=True)
        if internal_type == "Surface-Ship":
            self.set_broadcasting(True)
        elif internal_type != "Surface-Ship":
            self.set_broadcasting(False)
        self.update_route_visibility()


    def update_from_textboxes(self):
        # CONTACT NAME
        self.name_entered = self.contact_name_textbox.get_text().strip()

        # LATITUDE
        try:
            value = float(self.sub_lat_textbox.get_text())
            if -90 <= value <= 90:
                self.lat_entered = value
                self.sub_lat_textbox.set_text(f"{value:.5f}")
            else:
                self.sub_lat_textbox.set_text("")
        except:
            self.sub_lat_textbox.set_text("")

        # LONGITUDE
        try:
            value = float(self.sub_long_textbox.get_text())
            if -180 <= value <= 180:
                self.long_entered = value
                self.sub_long_textbox.set_text(f"{value:.5f}")
            else:
                self.sub_long_textbox.set_text("")
        except:
            self.sub_long_textbox.set_text("")

        # RANGE
        try:
            value = float(self.sub_range_textbox.get_text())
            if 0 <= value <= 250:
                self.range_entered = value
                self.sub_range_textbox.set_text(f"{value:.1f}")
            else:
                self.sub_range_textbox.set_text("")
        except:
            self.sub_range_textbox.set_text("")

        # SPEED
        try:
            value = float(self.sub_speed_textbox.get_text())
            if 0 <= value <= 50:
                self.speed_entered = value
                self.sub_speed_textbox.set_text(f"{int(value)}")
            else:
                self.sub_speed_textbox.set_text("")
        except:
            self.sub_speed_textbox.set_text("")

        # DEPTH
        try:
            value = float(self.sub_depth_textbox.get_text())
            if 0 <= value <= 10000:
                self.depth_entered = value
                self.sub_depth_textbox.set_text(f"{int(value)}")
            else:
                self.sub_depth_textbox.set_text("")
        except:
            self.sub_depth_textbox.set_text("")

        # BEARING
        try:
            value = float(self.sub_bearing_textbox.get_text())
            if 0 <= value <= 360:
                self.bearing_entered = value
                self.sub_bearing_textbox.set_text(f"{int(value):03d}")
            else:
                self.sub_bearing_textbox.set_text("")
        except:
            self.sub_bearing_textbox.set_text("")

        self.route_text_entered = self.route_textbox.get_text().strip()
        self.shadow_target_entered = self.shadow_target_textbox.get_text().strip()




# Main buttons outside the panel
start_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((656, 625), (627, 30)),
    text="START",
    manager=manager
)
save_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((10, 625), (313, 30)),
    text="SAVE",
    manager=manager
)
load_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((333, 625), (313, 30)),
    text="LOAD",
    manager=manager
)
simulator_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((1293, 625), (617, 30)),
    text="X-Plane",
    manager=manager
)
civilian_traffic_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((10, 665), (220, 30)),
    text="ADD CIV TRAFFIC",
    manager=manager
)
whale_traffic_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((240, 665), (220, 30)),
    text="ADD WHALES",
    manager=manager
)
civilian_lat_label = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((480, 668), (58, 24)),
    text="CIV LAT",
    manager=manager
)
civilian_lat_entry = pygame_gui.elements.UITextEntryLine(
    relative_rect=pygame.Rect((542, 665), (112, 30)),
    manager=manager
)
civilian_lon_label = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((664, 668), (58, 24)),
    text="CIV LON",
    manager=manager
)
civilian_lon_entry = pygame_gui.elements.UITextEntryLine(
    relative_rect=pygame.Rect((726, 665), (112, 30)),
    manager=manager
)
civilian_lat_entry.set_allowed_characters(['0','1','2','3','4','5','6','7','8','9','.','-'])
civilian_lon_entry.set_allowed_characters(['0','1','2','3','4','5','6','7','8','9','.','-'])
multiplayer_role_dropdown = pygame_gui.elements.UIDropDownMenu(
    options_list=["OFF", "SERVER", "HOST", "JOIN"],
    starting_option="OFF",
    relative_rect=pygame.Rect((10, 705), (150, 30)),
    manager=manager
)
multiplayer_callsign_entry = pygame_gui.elements.UITextEntryLine(
    relative_rect=pygame.Rect((170, 705), (210, 30)),
    manager=manager
)
multiplayer_callsign_entry.set_text(MULTIPLAYER_CALLSIGN)
multiplayer_aircraft_dropdown = force_dropdown_down(pygame_gui.elements.UIDropDownMenu(
    options_list=MULTIPLAYER_AIRCRAFT_TYPES,
    starting_option=MULTIPLAYER_AIRCRAFT_TYPE,
    relative_rect=pygame.Rect((390, 705), (300, 30)),
    manager=manager
))
multiplayer_player_type_dropdown = pygame_gui.elements.UIDropDownMenu(
    options_list=MULTIPLAYER_PLAYER_TYPES,
    starting_option=MULTIPLAYER_PLAYER_TYPE,
    relative_rect=pygame.Rect((700, 705), (150, 30)),
    manager=manager
)
multiplayer_team_dropdown = pygame_gui.elements.UIDropDownMenu(
    options_list=MULTIPLAYER_TEAMS,
    starting_option=MULTIPLAYER_TEAM,
    relative_rect=pygame.Rect((860, 705), (150, 30)),
    manager=manager
)
multiplayer_password_label = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((10, 745), (150, 24)),
    text="CONTACT PW",
    manager=manager
)
multiplayer_password_entry = pygame_gui.elements.UITextEntryLine(
    relative_rect=pygame.Rect((170, 745), (210, 30)),
    manager=manager
)
try:
    multiplayer_password_entry.set_text_hidden(True)
except AttributeError:
    pass
multiplayer_contact_dropdown = force_dropdown_down(pygame_gui.elements.UIDropDownMenu(
    options_list=ownship_control_contact_options,
    starting_option="Auto",
    relative_rect=pygame.Rect((390, 705), (300, 30)),
    manager=manager
))
multiplayer_contact_dropdown.hide()
multiplayer_status_label = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((1020, 705), (873, 30)),
    text="MP OFF",
    manager=manager
)

# Updated register_ui_element calls
register_ui_element(save_button, save_button.relative_rect)
register_ui_element(load_button, load_button.relative_rect)
register_ui_element(simulator_button, simulator_button.relative_rect)
register_ui_element(start_button, start_button.relative_rect)
register_ui_element(civilian_traffic_button, civilian_traffic_button.relative_rect)
register_ui_element(whale_traffic_button, whale_traffic_button.relative_rect)
register_ui_element(civilian_lat_label, civilian_lat_label.relative_rect)
register_ui_element(civilian_lat_entry, civilian_lat_entry.relative_rect)
register_ui_element(civilian_lon_label, civilian_lon_label.relative_rect)
register_ui_element(civilian_lon_entry, civilian_lon_entry.relative_rect)
register_ui_element(multiplayer_role_dropdown, multiplayer_role_dropdown.relative_rect)
register_ui_element(multiplayer_callsign_entry, multiplayer_callsign_entry.relative_rect)
register_ui_element(multiplayer_aircraft_dropdown, multiplayer_aircraft_dropdown.relative_rect)
register_ui_element(multiplayer_player_type_dropdown, multiplayer_player_type_dropdown.relative_rect)
register_ui_element(multiplayer_team_dropdown, multiplayer_team_dropdown.relative_rect)
register_ui_element(multiplayer_password_label, multiplayer_password_label.relative_rect)
register_ui_element(multiplayer_password_entry, multiplayer_password_entry.relative_rect)
register_ui_element(multiplayer_contact_dropdown, multiplayer_contact_dropdown.relative_rect)
register_ui_element(multiplayer_status_label, multiplayer_status_label.relative_rect)

def internal_rect_to_screen_rect(base_rect):
    return pygame.Rect(
        display_viewport_rect.x + int(base_rect.x * display_scale),
        display_viewport_rect.y + int(base_rect.y * display_scale),
        max(1, int(base_rect.w * display_scale)),
        max(1, int(base_rect.h * display_scale))
    )


def resize_ui(new_width, new_height, base_width=1920, base_height=1080):
    """
    Rescales registered UI elements into the preserved-aspect viewport.
    """
    for element, base_rect in ui_elements:
        scaled_rect = internal_rect_to_screen_rect(base_rect)
        element.set_relative_position((scaled_rect.x, scaled_rect.y))
        element.set_dimensions((scaled_rect.w, scaled_rect.h))


def layout_top_mode_buttons(window_width, window_height):
    """Keep the map/radar controls aligned to the internal right-side display."""
    gap = 6
    button_height = 28
    mode_width = 76
    orientation_width = 108
    range_width = 90
    ships_width = 92
    lines_width = 94
    internal_x = int(INTERNAL_WIDTH * 0.505)
    internal_y = 10

    top_buttons = (
        (map_mode_button, mode_width),
        (radar_mode_button, mode_width),
        (nav_mode_button, mode_width),
        (radar_orientation_button, orientation_width),
        (radar_range_button, range_width),
        (bearing_lines_button, lines_width),
        (ship_inject_button, ships_width)
    )

    x = internal_x
    for button, width in top_buttons:
        rect = internal_rect_to_screen_rect(pygame.Rect(x, internal_y, width, button_height))
        button.set_relative_position((rect.x, rect.y))
        button.set_dimensions((rect.w, rect.h))
        x += width + gap



def sync_slot_range_circle_button_style(slot):
    if not hasattr(slot, "toggle_range_circle_button"):
        return
    range_circle = False
    if is_numeric_channel(getattr(slot, "selected", None)):
        selected_channel = int(slot.selected)
        for sono in globals().get("sono_array", []):
            if int(getattr(sono, "channel", -1)) == selected_channel:
                range_circle = getattr(sono, "range_circle", False)
                break
    new_colour = pygame.Color("#99C979") if range_circle else pygame.Color("#b13b3b")
    slot.toggle_range_circle_button.colours["normal_bg"] = new_colour
    slot.toggle_range_circle_button.colours["hovered_bg"] = new_colour
    slot.toggle_range_circle_button.colours["active_bg"] = new_colour
    slot.toggle_range_circle_button.rebuild()


def sync_stateful_button_styles():
    """Reapply runtime button colours after pygame_gui rebuilds on resize."""
    for function_name in (
        "sync_bearing_lines_button_style",
        "sync_arm_button_style",
        "sync_auto_buoy_button_style",
        "sync_ship_inject_button_style",
        "sync_torpedo_designate_button_style",
        "sync_contact_lines_button_style",
    ):
        sync_function = globals().get(function_name)
        if sync_function is not None:
            sync_function()

    if globals().get("selected_contact") is not None and "sync_ship_command_controls" in globals():
        sync_ship_command_controls()

    for row in globals().get("contact_define_row_array", []):
        if hasattr(row, "sync_broadcasting_checkbox"):
            row.sync_broadcasting_checkbox()

    for slot in globals().get("spectrogram_slot_array", []):
        sync_slot_range_circle_button_style(slot)
        for method_name in (
            "sync_difar_display_button_style",
            "sync_band_mode_button_style",
            "sync_bearing_lines_button_style",
            "sync_listen_button_style",
        ):
            sync_method = getattr(slot, method_name, None)
            if sync_method is not None:
                sync_method()


def sync_display_mode_control_visibility():
    if "in_menu" in globals() and in_menu:
        return
    radar_controls_visible = display_mode == "RADAR"
    for element in (radar_orientation_button, radar_range_button):
        if radar_controls_visible:
            element.show()
        else:
            element.hide()

def scale_rect(base_rect, screen, base_width=1920, base_height=1080):
    """
    Takes a rect designed for 1920x1080 and maps it into the current viewport.
    """
    return internal_rect_to_screen_rect(base_rect)


SETTINGS_RESOLUTION_OPTIONS = [
    "16:9 1920x1080",
    "16:9 1600x900",
    "16:9 1280x720",
    "16:10 1680x1050",
    "4:3 1440x1080",
    "21:9 1920x823"
]
SETTINGS_RESOLUTION_SIZES = {
    "16:9 1920x1080": (1920, 1080),
    "16:9 1600x900": (1600, 900),
    "16:9 1280x720": (1280, 720),
    "16:10 1680x1050": (1680, 1050),
    "4:3 1440x1080": (1440, 1080),
    "21:9 1920x823": (1920, 823)
}
SETTINGS_SOUND_OPTIONS = ["100%", "75%", "50%", "25%", "0%"]
master_sound_level = 1.0
settings_panel_visible = False


settings_button = pygame_gui.elements.UIButton(
    relative_rect=scale_rect(pygame.Rect((1868, 8), (42, 30)), screen),
    text="⚙",
    manager=manager,
    tool_tip_text="Settings"
)
settings_panel = pygame_gui.elements.UIPanel(
    relative_rect=scale_rect(pygame.Rect((1510, 48), (400, 300)), screen),
    manager=manager
)
settings_title = pygame_gui.elements.UILabel(
    relative_rect=scale_rect(pygame.Rect((1524, 56), (260, 24)), screen),
    text="SETTINGS",
    manager=manager
)
settings_close_button = pygame_gui.elements.UIButton(
    relative_rect=scale_rect(pygame.Rect((1846, 56), (50, 24)), screen),
    text="CLOSE",
    manager=manager
)
settings_resolution_label = pygame_gui.elements.UILabel(
    relative_rect=scale_rect(pygame.Rect((1524, 92), (110, 24)), screen),
    text="RESOLUTION",
    manager=manager
)
settings_resolution_dropdown = pygame_gui.elements.UIDropDownMenu(
    options_list=SETTINGS_RESOLUTION_OPTIONS,
    starting_option="16:9 1920x1080",
    relative_rect=scale_rect(pygame.Rect((1640, 90), (236, 28)), screen),
    manager=manager
)
settings_sound_label = pygame_gui.elements.UILabel(
    relative_rect=scale_rect(pygame.Rect((1524, 130), (110, 24)), screen),
    text="SOUND",
    manager=manager
)
settings_sound_dropdown = pygame_gui.elements.UIDropDownMenu(
    options_list=SETTINGS_SOUND_OPTIONS,
    starting_option="100%",
    relative_rect=scale_rect(pygame.Rect((1640, 128), (120, 28)), screen),
    manager=manager
)
settings_simulator_label = pygame_gui.elements.UILabel(
    relative_rect=scale_rect(pygame.Rect((1524, 168), (110, 24)), screen),
    text="SIMULATOR",
    manager=manager
)
settings_simulator_dropdown = pygame_gui.elements.UIDropDownMenu(
    options_list=["X-Plane", "MSFS"],
    starting_option="X-Plane" if xplane == 1 else "MSFS",
    relative_rect=scale_rect(pygame.Rect((1640, 166), (120, 28)), screen),
    manager=manager
)
settings_callsign_label = pygame_gui.elements.UILabel(
    relative_rect=scale_rect(pygame.Rect((1524, 206), (110, 24)), screen),
    text="CALLSIGN",
    manager=manager
)
settings_callsign_entry = pygame_gui.elements.UITextEntryLine(
    relative_rect=scale_rect(pygame.Rect((1640, 204), (120, 28)), screen),
    manager=manager
)
settings_callsign_entry.set_text(MULTIPLAYER_CALLSIGN)
settings_aircraft_label = pygame_gui.elements.UILabel(
    relative_rect=scale_rect(pygame.Rect((1524, 244), (110, 24)), screen),
    text="AIRCRAFT",
    manager=manager
)
settings_aircraft_dropdown = pygame_gui.elements.UIDropDownMenu(
    options_list=MULTIPLAYER_AIRCRAFT_TYPES,
    starting_option=MULTIPLAYER_AIRCRAFT_TYPE,
    relative_rect=scale_rect(pygame.Rect((1640, 242), (120, 28)), screen),
    manager=manager
)
settings_version_label = pygame_gui.elements.UILabel(
    relative_rect=scale_rect(pygame.Rect((1770, 204), (126, 28)), screen),
    text=f"v{APP_VERSION}",
    manager=manager
)
settings_status_label = pygame_gui.elements.UILabel(
    relative_rect=scale_rect(pygame.Rect((1770, 242), (126, 28)), screen),
    text=f"{MULTIPLAYER_CALLSIGN} {MULTIPLAYER_AIRCRAFT_TYPE}",
    manager=manager
)
settings_update_button = pygame_gui.elements.UIButton(
    relative_rect=scale_rect(pygame.Rect((1524, 282), (116, 28)), screen),
    text="UPDATE",
    manager=manager,
    tool_tip_text="Check GitHub Releases and install the latest vASW build."
)
settings_update_status_label = pygame_gui.elements.UILabel(
    relative_rect=scale_rect(pygame.Rect((1650, 282), (246, 28)), screen),
    text="GitHub releases",
    manager=manager
)
settings_elements = [
    settings_panel,
    settings_title,
    settings_close_button,
    settings_resolution_label,
    settings_resolution_dropdown,
    settings_sound_label,
    settings_sound_dropdown,
    settings_simulator_label,
    settings_simulator_dropdown,
    settings_callsign_label,
    settings_callsign_entry,
    settings_aircraft_label,
    settings_aircraft_dropdown,
    settings_version_label,
    settings_status_label,
    settings_update_button,
    settings_update_status_label
]

register_ui_element(settings_button, pygame.Rect((1868, 8), (42, 30)))
register_ui_element(settings_panel, pygame.Rect((1510, 48), (400, 300)))
register_ui_element(settings_title, pygame.Rect((1524, 56), (260, 24)))
register_ui_element(settings_close_button, pygame.Rect((1846, 56), (50, 24)))
register_ui_element(settings_resolution_label, pygame.Rect((1524, 92), (110, 24)))
register_ui_element(settings_resolution_dropdown, pygame.Rect((1640, 90), (236, 28)))
register_ui_element(settings_sound_label, pygame.Rect((1524, 130), (110, 24)))
register_ui_element(settings_sound_dropdown, pygame.Rect((1640, 128), (120, 28)))
register_ui_element(settings_simulator_label, pygame.Rect((1524, 168), (110, 24)))
register_ui_element(settings_simulator_dropdown, pygame.Rect((1640, 166), (120, 28)))
register_ui_element(settings_callsign_label, pygame.Rect((1524, 206), (110, 24)))
register_ui_element(settings_callsign_entry, pygame.Rect((1640, 204), (120, 28)))
register_ui_element(settings_aircraft_label, pygame.Rect((1524, 244), (110, 24)))
register_ui_element(settings_aircraft_dropdown, pygame.Rect((1640, 242), (120, 28)))
register_ui_element(settings_version_label, pygame.Rect((1770, 204), (126, 28)))
register_ui_element(settings_status_label, pygame.Rect((1770, 242), (126, 28)))
register_ui_element(settings_update_button, pygame.Rect((1524, 282), (116, 28)))
register_ui_element(settings_update_status_label, pygame.Rect((1650, 282), (246, 28)))

update_popup_visible = False
update_popup_release = None
update_check_started = False
update_check_completed = False
update_check_result = None
update_check_error = None
update_check_popup_shown = False
update_check_start_time = time.time() + 2.5
update_check_lock = Lock()

update_popup_panel = pygame_gui.elements.UIPanel(
    relative_rect=scale_rect(pygame.Rect((660, 390), (600, 250)), screen),
    manager=manager
)
update_popup_title = pygame_gui.elements.UILabel(
    relative_rect=scale_rect(pygame.Rect((690, 414), (540, 34)), screen),
    text="UPDATE AVAILABLE",
    manager=manager
)
update_popup_message = pygame_gui.elements.UILabel(
    relative_rect=scale_rect(pygame.Rect((700, 470), (520, 72)), screen),
    text="A newer vASW version is available.",
    manager=manager
)
update_popup_update_button = pygame_gui.elements.UIButton(
    relative_rect=scale_rect(pygame.Rect((790, 568), (150, 34)), screen),
    text="UPDATE",
    manager=manager
)
update_popup_later_button = pygame_gui.elements.UIButton(
    relative_rect=scale_rect(pygame.Rect((980, 568), (150, 34)), screen),
    text="LATER",
    manager=manager
)
update_popup_elements = [
    update_popup_panel,
    update_popup_title,
    update_popup_message,
    update_popup_update_button,
    update_popup_later_button
]

register_ui_element(update_popup_panel, pygame.Rect((660, 390), (600, 250)))
register_ui_element(update_popup_title, pygame.Rect((690, 414), (540, 34)))
register_ui_element(update_popup_message, pygame.Rect((700, 470), (520, 72)))
register_ui_element(update_popup_update_button, pygame.Rect((790, 568), (150, 34)))
register_ui_element(update_popup_later_button, pygame.Rect((980, 568), (150, 34)))

for element in update_popup_elements:
    element.hide()



duplicate_contact_popup_visible = False
duplicate_contact_popup_panel = pygame_gui.elements.UIPanel(
    relative_rect=pygame.Rect((690, 420), (540, 190)),
    manager=manager
)
duplicate_contact_popup_title = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((20, 18), (500, 30)),
    text="Duplicate contact name",
    manager=manager,
    container=duplicate_contact_popup_panel
)
duplicate_contact_popup_message = pygame_gui.elements.UITextBox(
    relative_rect=pygame.Rect((24, 58), (492, 64)),
    html_text="A contact with this name already exists. Override it?",
    manager=manager,
    container=duplicate_contact_popup_panel
)
duplicate_contact_override_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((118, 136), (130, 34)),
    text="OVERRIDE",
    manager=manager,
    container=duplicate_contact_popup_panel
)
duplicate_contact_cancel_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((292, 136), (130, 34)),
    text="CANCEL",
    manager=manager,
    container=duplicate_contact_popup_panel
)
duplicate_contact_popup_elements = [
    duplicate_contact_popup_panel,
    duplicate_contact_popup_title,
    duplicate_contact_popup_message,
    duplicate_contact_override_button,
    duplicate_contact_cancel_button
]
register_ui_element(duplicate_contact_popup_panel, pygame.Rect((690, 420), (540, 190)))
for element in duplicate_contact_popup_elements:
    element.hide()

def parse_version_tag(tag):
    nums = [int(part) for part in re.findall(r"\d+", str(tag or ""))[:3]]
    while len(nums) < 3:
        nums.append(0)
    return tuple(nums)


def latest_release_metadata():
    request = urllib.request.Request(
        GITHUB_RELEASE_API,
        headers={
            "Accept": "application/vnd.github+json",
            "User-Agent": f"vASW/{APP_VERSION}"
        }
    )
    with urllib.request.urlopen(request, timeout=20) as response:
        return json.loads(response.read().decode("utf-8"))


def set_update_popup_visible(visible, release=None):
    global update_popup_visible, update_popup_release
    update_popup_visible = bool(visible)
    update_popup_release = release if visible else None
    if release:
        latest_tag = str(release.get("tag_name", "new version"))
        update_popup_message.set_text(f"{latest_tag} is available. Current version is v{APP_VERSION}.")
    for element in update_popup_elements:
        if update_popup_visible:
            element.show()
        else:
            element.hide()


def run_update_check_worker():
    global update_check_completed, update_check_result, update_check_error
    try:
        release = latest_release_metadata()
        latest_tag = str(release.get("tag_name", ""))
        if parse_version_tag(latest_tag) > parse_version_tag(APP_VERSION):
            result = release
        else:
            result = None
        with update_check_lock:
            update_check_result = result
            update_check_error = None
            update_check_completed = True
    except Exception as exc:
        with update_check_lock:
            update_check_result = None
            update_check_error = str(exc)
            update_check_completed = True
        print(f"Update check failed: {exc}")


def start_background_update_check():
    global update_check_started
    with update_check_lock:
        if update_check_started:
            return
        update_check_started = True
    Thread(target=run_update_check_worker, daemon=True).start()


def maybe_show_update_popup():
    global update_check_popup_shown
    with update_check_lock:
        release = update_check_result if update_check_completed else None
        already_shown = update_check_popup_shown
        if release is not None and not already_shown:
            update_check_popup_shown = True
    if release is not None and not already_shown:
        latest_tag = str(release.get("tag_name", "new"))
        settings_update_status_label.set_text(f"Update {latest_tag} available")
        set_update_popup_visible(True, release)


def select_release_asset(release):
    assets = release.get("assets", []) or []
    zip_assets = [asset for asset in assets if str(asset.get("name", "")).lower().endswith(".zip")]
    for asset in zip_assets:
        name = str(asset.get("name", "")).lower()
        if any(keyword.lower() in name for keyword in UPDATE_ASSET_KEYWORDS):
            return asset
    return zip_assets[0] if zip_assets else None


def find_extracted_app_dir(root_dir):
    direct = os.path.join(root_dir, "vASW.exe")
    if os.path.exists(direct):
        return root_dir
    for current_root, _, files in os.walk(root_dir):
        if "vASW.exe" in files:
            return current_root
    return None


def write_update_batch(source_dir, app_dir, temp_dir):
    script_path = os.path.join(temp_dir, "apply_vasw_update.bat")
    exe_path = os.path.join(app_dir, "vASW.exe")
    lines = [
        "@echo off",
        "setlocal",
        "echo Updating vASW...",
        "timeout /t 2 /nobreak >nul",
        f'xcopy "{source_dir}\\*" "{app_dir}\\" /E /I /Y >nul',
        f'start "" "{exe_path}"',
        "endlocal"
    ]
    with open(script_path, "w", encoding="ascii", errors="ignore") as script_file:
        script_file.write("\n".join(lines))
    return script_path


def download_release_asset(asset, zip_path):
    request = urllib.request.Request(
        asset["browser_download_url"],
        headers={"User-Agent": f"vASW/{APP_VERSION}"}
    )
    with urllib.request.urlopen(request, timeout=60) as response, open(zip_path, "wb") as out_file:
        while True:
            chunk = response.read(1024 * 256)
            if not chunk:
                break
            out_file.write(chunk)


def check_and_install_update():
    settings_update_status_label.set_text("Checking...")
    try:
        release = latest_release_metadata()
        latest_tag = str(release.get("tag_name", ""))
        if parse_version_tag(latest_tag) <= parse_version_tag(APP_VERSION):
            settings_update_status_label.set_text(f"Current v{APP_VERSION}")
            return
        asset = select_release_asset(release)
        if asset is None:
            settings_update_status_label.set_text("No Windows zip")
            return
        if not getattr(sys, "frozen", False):
            settings_update_status_label.set_text(f"Latest {latest_tag}; packaged app only")
            return

        settings_update_status_label.set_text(f"Downloading {latest_tag}")
        temp_dir = tempfile.mkdtemp(prefix="vasw_update_")
        zip_path = os.path.join(temp_dir, str(asset.get("name", "vASW-update.zip")))
        download_release_asset(asset, zip_path)

        extract_dir = os.path.join(temp_dir, "extracted")
        os.makedirs(extract_dir, exist_ok=True)
        with zipfile.ZipFile(zip_path, "r") as zip_file:
            zip_file.extractall(extract_dir)

        source_dir = find_extracted_app_dir(extract_dir)
        if source_dir is None:
            settings_update_status_label.set_text("Bad update zip")
            return

        app_dir = os.path.dirname(sys.executable)
        script_path = write_update_batch(source_dir, app_dir, temp_dir)
        settings_update_status_label.set_text("Restarting...")
        subprocess.Popen(["cmd", "/c", "start", "", script_path], shell=False)
        stop_listen_audio()
        pygame.quit()
        sys.exit(0)
    except Exception as exc:
        settings_update_status_label.set_text("Update failed")
        print(f"Update failed: {exc}")

def set_settings_panel_visible(visible):
    global settings_panel_visible
    settings_panel_visible = bool(visible)
    for element in settings_elements:
        if settings_panel_visible:
            element.show()
        else:
            element.hide()


def sync_settings_fields():
    settings_simulator_dropdown.selected_option = "X-Plane" if xplane == 1 else "MSFS"
    settings_callsign_entry.set_text(MULTIPLAYER_CALLSIGN)
    settings_version_label.set_text(f"v{APP_VERSION}")
    settings_status_label.set_text(f"{MULTIPLAYER_CALLSIGN} {multiplayer_platform_label()}")


def apply_sound_level(option):
    global master_sound_level
    try:
        master_sound_level = max(0.0, min(1.0, float(str(option).replace("%", "")) / 100.0))
    except ValueError:
        master_sound_level = 1.0
    if "launch_sound" in globals() and launch_sound is not None:
        launch_sound.set_volume(master_sound_level)


def apply_resolution_option(option):
    global screen_width, screen_height, screen
    width, height = SETTINGS_RESOLUTION_SIZES.get(str(option), (screen_width, screen_height))
    screen_width, screen_height = width, height
    screen = pygame.display.set_mode((screen_width, screen_height), pygame.RESIZABLE)
    update_display_viewport()
    manager.set_window_resolution((screen_width, screen_height))
    refresh_resolution_dependent_fonts()
    resize_ui(screen_width, screen_height)
    layout_top_mode_buttons(screen_width, screen_height)
    sync_stateful_button_styles()
    radar_terrain_cache["key"] = None


def set_simulator_mode(mode):
    global xplane, lat, long, alt, sm, aq
    if mode == "MSFS":
        if xplane == 1:
            xplane = 0
            lat = None
            long = None
            alt = None
            simulator_button.set_text("MSFS")
            try:
                sm = SimConnect()
                aq = AircraftRequests(sm, _time=2000)
            except ConnectionError:
                sm = None
                aq = None
                print("Flight Simulator not detected")
            clear_xplane_ship_export()
    else:
        if xplane == 0:
            xplane = 1
            simulator_button.set_text("X-Plane")
            remove_all_injected_ships()
    settings_simulator_dropdown.selected_option = "X-Plane" if xplane == 1 else "MSFS"
    sync_ship_inject_button_style()


set_settings_panel_visible(False)

# ---------------------------------------------------------------------------
# Menu/config state
# ---------------------------------------------------------------------------
json_path = r"aircraft_position.json"
last_xplane_read_time = 0
xplane_read_interval = 0.1
last_xplane_data = None

# Global list to hold all rows
contact_define_row_array = []


def reflow_contact_define_rows():
    x = 5
    for row in contact_define_row_array:
        row.row_panel.set_relative_position((x, 10))
        x = row.row_panel.rect.right + 10
    update_contact_define_scroll_area()


def ensure_contact_define_row_exists():
    if not contact_define_row_array:
        row = ContactDefineRow(10, manager, contact_define_container)
        contact_define_row_array.append(row)
        update_contact_define_scroll_area()


def row_contact_name(row):
    try:
        row.update_from_textboxes()
    except Exception:
        pass
    return str(getattr(row, "name_entered", "") or row.contact_name_textbox.get_text() or "").strip()


def delete_contact_define_row(row, delete_live=True):
    if row not in contact_define_row_array:
        return False
    contact_name = row_contact_name(row)
    if delete_live and contact_name:
        for contact in list(contacts):
            if str(getattr(contact, "name", "")) == contact_name:
                delete_contact(contact, delete_rows=False)
    row.row_panel.kill()
    contact_define_row_array.remove(row)
    ensure_contact_define_row_exists()
    reflow_contact_define_rows()
    return True


def delete_matching_contact_rows(contact):
    contact_name = str(getattr(contact, "name", "") or "").strip()
    if not contact_name:
        return 0
    removed = 0
    for row in list(contact_define_row_array):
        if row_contact_name(row) == contact_name:
            row.row_panel.kill()
            contact_define_row_array.remove(row)
            removed += 1
    ensure_contact_define_row_exists()
    reflow_contact_define_rows()
    return removed


def hide_contact_context_controls():
    contact_context_panel.hide()
    torpedo_designate_button.hide()
    contact_lines_button.hide()
    hide_ship_command_controls()
    contact_delete_button.hide()
    contact_context_close_button.hide()
    contact_type_dropdown.hide()
    contact_class_dropdown.hide()
    contact_status_dropdown.hide()
    contact_country_dropdown.hide()


def delete_contact(contact, delete_rows=True):
    global selected_contact, torpedo_designated_contact, multiplayer_last_state_broadcast
    if contact is None:
        return False
    track_number = getattr(contact, "track_number", None)
    if track_number is not None:
        remove_injected_ship(track_number)
    if torpedo_designated_contact is contact:
        torpedo_designated_contact = None
    for torpedo in torp_array:
        if getattr(torpedo, "target", None) is contact:
            torpedo.target = None
    if contact in contacts:
        contacts.remove(contact)
    if delete_rows:
        delete_matching_contact_rows(contact)
    if selected_contact is contact:
        selected_contact = None
        hide_contact_context_controls()
    multiplayer_last_state_broadcast = 0.0
    print(f"Deleted contact {getattr(contact, 'name', 'Contact')} track {track_number if track_number is not None else '?'}")
    return True



def contact_by_name(name):
    wanted = str(name or "").strip()
    if not wanted:
        return None
    for contact in contacts:
        if str(getattr(contact, "name", "") or "").strip() == wanted:
            return contact
    return None


def set_duplicate_contact_popup_visible(visible, row=None, name=""):
    global duplicate_contact_popup_visible, duplicate_contact_pending_row
    duplicate_contact_popup_visible = bool(visible)
    duplicate_contact_pending_row = row if visible else None
    if visible:
        duplicate_contact_popup_message.set_text(f"Contact {name} already exists. Override it?")
    for element in duplicate_contact_popup_elements:
        if duplicate_contact_popup_visible:
            element.show()
        else:
            element.hide()


def define_contact_from_row(row, override_duplicate=False):
    try:
        row.update_from_textboxes()
    except Exception:
        pass
    print(row.name_entered, row.lat_entered, row.long_entered, row.range_entered, row.class_entered, row.speed_entered, row.depth_entered, row.bearing_entered)
    if not (row.name_entered and row.lat_entered is not None and row.long_entered is not None
            and row.range_entered is not None and row.internal_type_entered and row.internal_class_entered
            and row.speed_entered is not None and row.depth_entered is not None and row.bearing_entered is not None):
        return False

    existing_contact = contact_by_name(row.name_entered)
    if existing_contact is not None and not override_duplicate:
        set_duplicate_contact_popup_visible(True, row, row.name_entered)
        return False
    if existing_contact is not None and override_duplicate:
        delete_contact(existing_contact)

    spawn_lat, spawn_lon = random_point_within_range_nm(row.lat_entered, row.long_entered, row.range_entered)
    new_contact = Contact(
        name=row.name_entered,
        tones=[],
        contact_lat=spawn_lat,
        contact_long=spawn_lon,
        speed=row.speed_entered,
        depth=row.depth_entered,
        bearing=row.bearing_entered
    )
    resolved_internal_class = (
        resolve_submarine_class_selection(row.internal_class_entered)
        if row.internal_type_entered == "Sub-surface" else row.internal_class_entered
    )
    new_contact.internal_type = row.internal_type_entered
    new_contact.internal_class = resolved_internal_class
    new_contact.broadcasting = row.broadcasting_entered
    new_contact.team = row.team_entered
    new_contact.shadow_target_name = row.shadow_target_textbox.get_text().strip()
    new_contact.shadow_distance_nm = 5.0
    new_contact.model_library = row.selected_model_library
    if row.selected_model != "Auto":
        new_contact.gaist_model_title = row.selected_model

    acoustic_class = resolved_internal_class if row.internal_type_entered == "Sub-surface" else ""
    contact_class = sub_classes.get(acoustic_class)
    if contact_class is not None:
        sub_instance = contact_class(
            name=new_contact.name,
            contact_lat=new_contact.contact_lat,
            contact_long=new_contact.contact_long,
            speed=new_contact.speed,
            depth=new_contact.depth,
            bearing=new_contact.bearing
        )
        new_contact.tones = sub_instance.tones
    elif row.internal_type_entered == "Surface-Ship":
        if row.internal_class_entered == "Civilian":
            new_contact.classification_type = "Surface-Ship"
            new_contact.classification_class = "Civilian"
            new_contact.identity_status = "N"
            new_contact.operator_classified = True
        else:
            reset_contact_classification(new_contact)
        new_contact.detected = row.internal_class_entered == "Civilian"
        gaist_model_title_for_contact(new_contact)
        apply_surface_ship_acoustic_profile(new_contact)
    elif row.internal_type_entered == "Biological":
        if row.internal_class_entered == "Whale":
            new_contact.tones = whale_acoustic_profile()
            new_contact.acoustic_profile = "Whale"
        new_contact.classification_type = "Biological"
        new_contact.classification_class = row.internal_class_entered
        new_contact.identity_status = "N"
        new_contact.operator_classified = True
        new_contact.detected = False

    row.route_text_entered = row.route_textbox.get_text().strip()
    row.shadow_target_entered = row.shadow_target_textbox.get_text().strip()
    new_contact.shadow_target_name = row.shadow_target_entered
    new_contact.shadow_distance_nm = 5.0
    if row.is_route_enabled() and row.route_text_entered:
        assign_ship_route_from_text(new_contact, row.route_text_entered)

    contacts.append(new_contact)
    if multiplayer_role == "JOIN" and multiplayer_contact_password_ok():
        send_multiplayer_contact_contribution()
    update_all_contact_shadow_following()
    print(
        f"Added contact: {new_contact.name}, Track #{new_contact.track_number}, "
        f"Tones: {len(new_contact.tones)}, spawned {haversine(row.lat_entered, row.long_entered, spawn_lat, spawn_lon):.2f} NM from requested point"
    )
    last_row = contact_define_row_array[-1]
    new_x = last_row.row_panel.rect.right + 10
    new_row = ContactDefineRow(y=0, manager=manager, container=contact_define_container)
    new_row.row_panel.set_relative_position((new_x, 10))
    contact_define_row_array.append(new_row)
    update_contact_define_scroll_area()
    return True

def draw_menu():
    # Fill menu background
    menu_surface.fill((5,5,10))

    global contact_define_row_array
    refresh_control_contact_dropdown()
    update_multiplayer_platform_selector_visibility()

    # If no rows exist, create the first one
    if not contact_define_row_array:
        first_row = ContactDefineRow(10, manager, contact_define_container)
        contact_define_row_array.append(first_row)
        update_contact_define_scroll_area()


def draw_start_loading_progress(progress, status_text="Loading simulation"):
    pygame.event.pump()
    progress = max(0.0, min(1.0, float(progress or 0.0)))
    pygame.mouse.set_visible(progress >= 1.0)

    screen.fill((4, 8, 12))

    panel_width = min(max(360, int(screen_width * 0.42)), screen_width - 80)
    panel_height = 126
    panel_rect = pygame.Rect(0, 0, panel_width, panel_height)
    panel_rect.center = (screen_width // 2, screen_height // 2)
    pygame.draw.rect(screen, (12, 18, 24), panel_rect, border_radius=8)
    pygame.draw.rect(screen, (70, 92, 110), panel_rect, width=1, border_radius=8)

    title_font = scaled_sys_font(22, bold=True)
    label_font = scaled_sys_font(13)
    title_surface = title_font.render("LOADING SIMULATION", True, (232, 242, 245))
    status_surface = label_font.render(str(status_text), True, (170, 190, 198))
    screen.blit(title_surface, (panel_rect.x + 24, panel_rect.y + 20))
    screen.blit(status_surface, (panel_rect.x + 24, panel_rect.y + 50))

    bar_rect = pygame.Rect(panel_rect.x + 24, panel_rect.y + 82, panel_rect.w - 48, 16)
    fill_rect = bar_rect.copy()
    fill_rect.width = max(4, int(bar_rect.width * progress))
    pygame.draw.rect(screen, (34, 44, 52), bar_rect, border_radius=4)
    pygame.draw.rect(screen, (92, 205, 132), fill_rect, border_radius=4)
    pygame.draw.rect(screen, (95, 118, 130), bar_rect, width=1, border_radius=4)

    pygame.display.update()


def set_menu_visible(visible):
    """Show or hide every pygame_gui element that belongs to the start menu."""
    if visible:
        contact_panel.show()
        contact_define_container.show()
        start_button.show()
        save_button.show()
        load_button.show()
        simulator_button.show()
        civilian_traffic_button.show()
        whale_traffic_button.show()
        civilian_lat_label.show()
        civilian_lat_entry.show()
        civilian_lon_label.show()
        civilian_lon_entry.show()
        multiplayer_role_dropdown.show()
        multiplayer_callsign_entry.show()
        multiplayer_player_type_dropdown.show()
        multiplayer_team_dropdown.show()
        multiplayer_password_label.show()
        multiplayer_password_entry.show()
        refresh_control_contact_dropdown()
        update_multiplayer_platform_selector_visibility()
        multiplayer_status_label.show()
        map_mode_button.hide()
        radar_mode_button.hide()
        nav_mode_button.hide()
        radar_orientation_button.hide()
        radar_range_button.hide()
        ship_inject_button.hide()
        if "xbt_tab_button" in globals():
            xbt_tab_button.hide()
        if "xbt_panel_close_button" in globals():
            xbt_panel_close_button.hide()
        if "xbt_raytrace_button" in globals():
            xbt_raytrace_button.hide()
        if "xbt_raytrace_clear_button" in globals():
            xbt_raytrace_clear_button.hide()
        if "xbt_panel_select_dropdown" in globals() and xbt_panel_select_dropdown is not None:
            xbt_panel_select_dropdown.hide()
        contact_context_panel.hide()
        if "nav_elements" in globals():
            for element in nav_elements:
                element.hide()
        if "torpedo_designate_button" in globals():
            torpedo_designate_button.hide()
        if "contact_context_close_button" in globals():
            contact_context_close_button.hide()
        if "contact_type_dropdown" in globals():
            contact_type_dropdown.hide()
        if "contact_class_dropdown" in globals():
            contact_class_dropdown.hide()
        if "contact_status_dropdown" in globals():
            contact_status_dropdown.hide()
        if "contact_country_dropdown" in globals():
            contact_country_dropdown.hide()
    else:
        contact_panel.hide()
        contact_define_container.hide()
        start_button.hide()
        save_button.hide()
        load_button.hide()
        simulator_button.hide()
        civilian_traffic_button.hide()
        whale_traffic_button.hide()
        civilian_lat_label.hide()
        civilian_lat_entry.hide()
        civilian_lon_label.hide()
        civilian_lon_entry.hide()
        multiplayer_role_dropdown.hide()
        multiplayer_callsign_entry.hide()
        multiplayer_aircraft_dropdown.hide()
        multiplayer_player_type_dropdown.hide()
        multiplayer_team_dropdown.hide()
        multiplayer_password_label.hide()
        multiplayer_password_entry.hide()
        multiplayer_contact_dropdown.hide()
        multiplayer_status_label.hide()
        map_mode_button.show()
        radar_mode_button.show()
        nav_mode_button.show()
        sync_display_mode_control_visibility()
        ship_inject_button.show()
        if "xbt_tab_button" in globals():
            xbt_tab_button.show()

    for row in contact_define_row_array:
        row.row_panel.show() if visible else row.row_panel.hide()


def start_game(progress_callback=None):
    global lat, long, alt, sub_x, sub_y, sm, aq, hdg

    if progress_callback is None:
        progress_callback = lambda _progress, _status: None

    submarine_latitude = 50
    submarine_longitude = 0

    progress_callback(0.46, "Positioning player")

    if player_is_ship_or_sub():
        controlled = selected_ownship_control_contact()
        if controlled is not None:
            lat = float(controlled.contact_lat)
            long = float(controlled.contact_long)
            hdg = float(getattr(controlled, "bearing", hdg) or hdg)
            alt = -float(getattr(controlled, "depth", 0.0) or 0.0) if player_is_submarine() else 0.0
        return

    if xplane == 1:
        progress_callback(0.62, "Reading aircraft position")
        json_path = r"aircraft_position.json"
        with open(json_path, "r") as f:
            data = json.load(f)

        lat = data["latitude"]
        long = data["longitude"]
        alt = data["altitude_ft"] 

    else:
        progress_callback(0.62, "Connecting to simulator")
        try:
            sm = SimConnect()
            aq = AircraftRequests(sm, _time=2000)
            sim_connected = True

        except ConnectionError:
            sm = None
            aq = None

            sim_connected = False

        if sm and aq is not None:
            lat = aq.get("PLANE_LATITUDE")
            long = aq.get("PLANE_LONGITUDE")
            alt = aq.get("INDICATED_ALTITUDE")
        else:
            lat = None
            long = None
            alt = None

    progress_callback(0.82, "Preparing traffic")
    ensure_random_whales()
    progress_callback(0.92, "Finalizing display")




# Function to count contacts within a radius
def contacts_within_radius(contacts, center_lat, center_lon, radius_nm):
    """
    contacts: list of Contact objects
    center_lat, center_lon: reference point in degrees
    radius_nm: radius in nautical miles
    """
    nearby = []
    for contact in contacts:
        dist = haversine(center_lat, center_lon, contact.latitude, contact.longitude)
        if dist <= radius_nm:
            nearby.append(contact)
    return nearby


active_sono_array = []
sono_array = []
xbt_array = []
xbt_profiles = {}
xbt_counter = 0
latest_xbt_profile = None

sonoDropped = False
player_pos = 0,0


sub_depth = submarine_depth 
global sonoArmed
sonoArmed = False 
detection_count = 0
indicated_angle_to_sub = 0
depth_uncorrected_range = 0
depth_corrected_range = 0
sub_speed = 0
target_coord = (random.randrange(100,500),random.randrange(100,400))
sub_speed_pixels_per_sec = sub_speed * 0.00277777777

# 1nm = 10px  
# 1kt = 1 nmi/hr = 10px/hr = 0.00277777777
# 
edge_margin = 50
target_reached_threshold = 50  # Pixels distance to consider target reached
last_capture_time = 0  # Track the last time saved a position
capture_interval = 1  # in seconds
biblically_accurate_prop_db_level = 0
duration = 60
selected_sonobuoy = 0
spec_width, spec_height = 480, 128



spectro_array = []
last_save_time = 0
submarine_code = None

# ---------------------------------------------------------------------------
# Coordinate helpers
# ---------------------------------------------------------------------------
# The map art is treated as a flat 8000x4000 world. These helpers convert
# between geographic coordinates and that map coordinate system.
lat_center, lon_center = 0, 0
scale = 50   # pixels per degree at zoom=1
zoom = 1.0
panning = False
last_mouse = (0, 0)




launch_sound = None
if not DEDICATED_HOST_MODE and pygame.mixer.get_init():
    try:
        launch_sound = pygame.mixer.Sound("assets/launch_sound.wav")
        launch_sound.set_volume(master_sound_level)
    except pygame.error as exc:
        print(f"[AUDIO] Launch sound disabled: {exc}")


def play_launch_sound():
    if launch_sound is None:
        return
    try:
        launch_sound.play()
    except pygame.error as exc:
        print(f"[AUDIO] Launch sound failed: {exc}")

thermocline_depth = 1000
seabed_depth = 2600


surface_temp = 20
seabed_temp = 4
max_bt_depth = 2600
temp_step = 130

depth_step = np.arange(0, max_bt_depth + temp_step, temp_step)  # 0, 20, 40, ..., 1000

temp_step = []
temp_profile = []


def pix_to_latlong(x,y):
    long_offset = (x / 8000) * 360
    lat_offset = (y / 4000) * 180

    long = long_offset - 180
    lat = 90 - lat_offset
    
    return (lat, long)


def append_fading_trail(obj, pos, max_points=90):
    if not hasattr(obj, "trail"):
        obj.trail = []

    point = pygame.Vector2(pos)
    if not obj.trail or obj.trail[-1].distance_to(point) > 0.2:
        obj.trail.append(point)
        obj.trail = obj.trail[-max_points:]


def draw_fading_trail(surface, points, translate_func, colour, width=2):
    if len(points) < 2:
        return

    point_count = len(points)
    for i in range(1, point_count):
        start = pygame.Vector2(translate_func(points[i - 1]))
        end = pygame.Vector2(translate_func(points[i]))
        if start.distance_to(end) > 250:
            continue

        alpha_scale = 0.25 + 0.75 * (i / point_count)
        faded_colour = (
            int(colour[0] * alpha_scale),
            int(colour[1] * alpha_scale),
            int(colour[2] * alpha_scale)
        )
        pygame.draw.line(
            surface,
            faded_colour,
            start,
            end,
            width
        )


def contact_display_colour(contact):
    status = getattr(contact, "identity_status", "P")
    return contact_identity_colours.get(status, contact_identity_colours["P"])


def draw_contact_direction_line(surface, center_pos, bearing_deg, rotation_deg=0, length=68, colour=(255, 90, 80), width=2):
    display_bearing = (float(bearing_deg) - rotation_deg) % 360
    start = pygame.Vector2(center_pos)
    end = pygame.Vector2(
        start.x + math.sin(math.radians(display_bearing)) * length,
        start.y - math.cos(math.radians(display_bearing)) * length
    )
    pygame.draw.line(surface, colour, start, end, width)


def map_overlay_symbol_scale(zoom=None):
    zoom = max(1.0, float(map_layer.zoom if zoom is None else zoom))
    return max(0.85, min(2.6, zoom ** 0.22))


def draw_map_contact_marker(surface, contact, screen_pos, zoom=1.0):
    contact_colour = contact_display_colour(contact)
    marker_pos = pygame.Vector2(screen_pos)
    marker_scale = map_overlay_symbol_scale(zoom)
    icon_size = int(24 * marker_scale)
    line_len = int(52 * marker_scale)
    line_width = max(1, int(round(2 * marker_scale)))
    draw_contact_direction_line(
        surface,
        marker_pos,
        getattr(contact, "bearing", 0),
        0,
        line_len,
        contact_colour,
        line_width
    )
    if torpedo_designated_contact is contact:
        pygame.draw.circle(surface, (255, 230, 90), marker_pos, int(18 * marker_scale), line_width)
        target_label = font.render("TGT", False, (255, 230, 90))
        surface.blit(target_label, (marker_pos.x + int(10 * marker_scale), marker_pos.y + int(8 * marker_scale)))
    return blit_contact_icon(surface, contact, marker_pos, icon_size)


splash_effects = []


def spawn_splash(world_pos):
    spray = []
    for index in range(10):
        angle = (index * 36) + random.uniform(-12, 12)
        length = random.uniform(12, 24)
        spray.append((angle, length))

    splash_effects.append({
        "pos": pygame.Vector2(world_pos),
        "start": time.time(),
        "duration": 0.9,
        "spray": spray
    })


def draw_splash_effects(surface):
    now = time.time()
    active_splashes = []

    for splash in splash_effects:
        age = now - splash["start"]
        duration = splash["duration"]
        if age >= duration:
            continue

        active_splashes.append(splash)
        progress = age / duration
        screen_pos = pygame.Vector2(map_layer.translate_point(splash["pos"]))
        alpha = max(0, int(220 * (1 - progress)))
        ring_colour = (210, 235, 255, alpha)
        spray_colour = (235, 245, 255, alpha)
        ring_radius = max(3, int((10 + 28 * progress) * map_layer.zoom))
        inner_radius = max(2, int((4 + 16 * progress) * map_layer.zoom))

        pygame.draw.circle(surface, ring_colour, screen_pos, ring_radius, 2)
        pygame.draw.circle(surface, ring_colour, screen_pos, inner_radius, 1)

        for angle, length in splash["spray"]:
            start_len = length * 0.2 * map_layer.zoom
            end_len = length * (0.4 + progress) * map_layer.zoom
            start = (
                screen_pos.x + math.cos(math.radians(angle)) * start_len,
                screen_pos.y + math.sin(math.radians(angle)) * start_len
            )
            end = (
                screen_pos.x + math.cos(math.radians(angle)) * end_len,
                screen_pos.y + math.sin(math.radians(angle)) * end_len
            )
            pygame.draw.line(surface, spray_colour, start, end, 1)

    splash_effects[:] = active_splashes


def iter_xbt_positions():
    seen = set()
    for profile in xbt_profiles.values():
        position = getattr(profile, "position", None)
        if position is not None:
            key = (round(float(position.x), 2), round(float(position.y), 2))
            if key not in seen:
                seen.add(key)
                yield pygame.Vector2(position.x, position.y), getattr(profile, "label", "XBT")

    for xbt_item in xbt_array:
        if hasattr(xbt_item, "x") and hasattr(xbt_item, "y"):
            position = pygame.Vector2(xbt_item.x, xbt_item.y)
        elif isinstance(xbt_item, (tuple, list)) and len(xbt_item) >= 2:
            position = pygame.Vector2(xbt_item[0], xbt_item[1])
        else:
            continue
        key = (round(float(position.x), 2), round(float(position.y), 2))
        if key not in seen:
            seen.add(key)
            yield position, "XBT"


def draw_xbt_map_icons(surface):
    for xbt_pos, label in iter_xbt_positions():
        screen_pos = map_layer.translate_point(xbt_pos)
        icon_rect = sonoB_surface.get_rect(center=screen_pos)
        surface.blit(sonoB_surface, icon_rect)
        label_surface = font.render(str(label), False, (0, 220, 235))
        surface.blit(label_surface, (screen_pos[0] + 9, screen_pos[1] - 10))


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

def latlong_to_pix(lat,long):
    if lat is not None:
        lat_offset = (lat * -1) + 90
        long_offset = long + 180
        converted_x = long_offset / 360 * 8000
        converted_y = lat_offset / 180 * 4000
        
        return (converted_x, converted_y)


def load_coastline_polylines(path=COASTLINE_FILE):
    """Load GeoJSON coastline geometry and cache bounds for quick radar drawing."""
    if not os.path.exists(path):
        return []

    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    polylines = []
    for feature in data.get("features", []):
        geometry = feature.get("geometry", {})
        geometry_type = geometry.get("type")
        coordinates = geometry.get("coordinates", [])

        if geometry_type == "LineString":
            lines = [coordinates]
        elif geometry_type == "MultiLineString":
            lines = coordinates
        else:
            continue

        for line in lines:
            points = []
            for coord in line:
                if len(coord) >= 2:
                    lon, lat = coord[0], coord[1]
                    points.append((float(lat), float(lon)))
            if len(points) >= 2:
                lats = [point[0] for point in points]
                lons = [point[1] for point in points]
                polylines.append({
                    "points": points,
                    "bounds": (min(lats), max(lats), min(lons), max(lons))
                })

    return polylines


def bounds_for_points(points):
    lats = [point[0] for point in points]
    lons = [point[1] for point in points]
    return (min(lats), max(lats), min(lons), max(lons))


def terrain_cell(value):
    return math.floor(value / TERRAIN_INDEX_CELL_DEG)


def build_terrain_index(geometries):
    index = {}

    for geometry in geometries:
        min_lat, max_lat, min_lon, max_lon = geometry["bounds"]
        min_lat_cell = terrain_cell(min_lat)
        max_lat_cell = terrain_cell(max_lat)
        min_lon_cell = terrain_cell(min_lon)
        max_lon_cell = terrain_cell(max_lon)

        for lat_cell in range(min_lat_cell, max_lat_cell + 1):
            for lon_cell in range(min_lon_cell, max_lon_cell + 1):
                index.setdefault((lat_cell, lon_cell), []).append(geometry)

    return index


def query_terrain_index(index, own_lat, own_lon, radar_range_nm):
    lat_margin = radar_range_nm / 60 + 0.25
    lon_scale = max(0.2, math.cos(math.radians(own_lat)))
    lon_margin = radar_range_nm / (60 * lon_scale) + 0.25

    min_lat_cell = terrain_cell(own_lat - lat_margin)
    max_lat_cell = terrain_cell(own_lat + lat_margin)
    min_lon_cell = terrain_cell(own_lon - lon_margin)
    max_lon_cell = terrain_cell(own_lon + lon_margin)

    candidates = []
    seen = set()
    for lat_cell in range(min_lat_cell, max_lat_cell + 1):
        for lon_cell in range(min_lon_cell, max_lon_cell + 1):
            for geometry in index.get((lat_cell, lon_cell), []):
                geometry_id = id(geometry)
                if geometry_id in seen:
                    continue

                seen.add(geometry_id)
                candidates.append(geometry)

    return candidates


def load_land_polygons(path=LAND_FILE):
    """Load land polygon exterior rings for filled terrain on the radar display."""
    if not os.path.exists(path):
        return []

    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    polygons = []
    for feature in data.get("features", []):
        geometry = feature.get("geometry", {})
        geometry_type = geometry.get("type")
        coordinates = geometry.get("coordinates", [])

        if geometry_type == "Polygon":
            polygon_rings = coordinates
        elif geometry_type == "MultiPolygon":
            polygon_rings = [ring for polygon in coordinates for ring in polygon]
        else:
            continue

        for ring in polygon_rings:
            points = []
            for coord in ring:
                if len(coord) >= 2:
                    lon, lat = coord[0], coord[1]
                    points.append((float(lat), float(lon)))
            if len(points) >= 3:
                polygons.append({
                    "points": points,
                    "bounds": bounds_for_points(points)
                })

    return polygons


def lon_to_raster_x(lon, width):
    return int((lon + 180) / 360 * width)


def lat_to_raster_y(lat, height):
    return int((90 - lat) / 180 * height)


def create_land_raster(polygons, size=LAND_RASTER_SIZE):
    if not polygons:
        return None

    width, height = size
    raster = pygame.Surface(size, pygame.SRCALPHA)
    land_fill = (32, 45, 105, 220)

    for polygon in polygons:
        raster_points = []
        last_point = None
        for lat, lon in polygon["points"]:
            point = (lon_to_raster_x(lon, width), lat_to_raster_y(lat, height))
            if point == last_point:
                continue
            raster_points.append(point)
            last_point = point

        if len(raster_points) >= 3:
            pygame.draw.polygon(raster, land_fill, raster_points)

    return raster


coastline_polylines = load_coastline_polylines()
land_polygons = load_land_polygons()
coastline_index = build_terrain_index(coastline_polylines)
land_index = build_terrain_index(land_polygons)
land_raster_surface = create_land_raster(land_polygons)


def point_in_polygon_latlon(lat_value, lon_value, polygon_points):
    inside = False
    count = len(polygon_points)
    if count < 3:
        return False
    j = count - 1
    for i in range(count):
        lat_i, lon_i = polygon_points[i]
        lat_j, lon_j = polygon_points[j]
        crosses = ((lat_i > lat_value) != (lat_j > lat_value))
        if crosses:
            lon_cross = (lon_j - lon_i) * (lat_value - lat_i) / ((lat_j - lat_i) or 1e-12) + lon_i
            if lon_value < lon_cross:
                inside = not inside
        j = i
    return inside


def point_is_land(lat_value, lon_value):
    for polygon in query_terrain_index(land_index, lat_value, lon_value, 1.0):
        min_lat, max_lat, min_lon, max_lon = polygon["bounds"]
        if not (min_lat <= lat_value <= max_lat and min_lon <= lon_value <= max_lon):
            continue
        if point_in_polygon_latlon(lat_value, lon_value, polygon["points"]):
            return True
    return False


def point_to_segment_distance_nm(lat_value, lon_value, start_point, end_point):
    ref_lat_rad = math.radians(lat_value)

    def to_xy(point):
        point_lat, point_lon = point
        return (
            (point_lon - lon_value) * 60.0 * max(0.05, math.cos(ref_lat_rad)),
            (point_lat - lat_value) * 60.0
        )

    ax, ay = to_xy(start_point)
    bx, by = to_xy(end_point)
    abx = bx - ax
    aby = by - ay
    denom = abx * abx + aby * aby
    if denom <= 1e-9:
        return math.hypot(ax, ay)
    t = max(0.0, min(1.0, -(ax * abx + ay * aby) / denom))
    closest_x = ax + abx * t
    closest_y = ay + aby * t
    return math.hypot(closest_x, closest_y)


def distance_to_nearest_coast_nm(lat_value, lon_value, search_nm=25.0):
    nearest = None
    for coastline in query_terrain_index(coastline_index, lat_value, lon_value, search_nm):
        points = coastline.get("points", [])
        for index in range(len(points) - 1):
            distance_nm = point_to_segment_distance_nm(lat_value, lon_value, points[index], points[index + 1])
            if nearest is None or distance_nm < nearest:
                nearest = distance_nm
    return nearest


def random_sea_point_near(origin_lat, origin_lon, min_range_nm=CIVILIAN_TRAFFIC_MIN_RANGE_NM, max_range_nm=CIVILIAN_TRAFFIC_MAX_RANGE_NM, min_offshore_nm=CIVILIAN_TRAFFIC_MIN_OFFSHORE_NM, attempts=160):
    fallback = None
    for _ in range(attempts):
        spawn_bearing = random.uniform(0, 360)
        spawn_range_nm = random.uniform(min_range_nm, max_range_nm)
        candidate_lat, candidate_lon = destination_from_bearing(origin_lat, origin_lon, spawn_bearing, spawn_range_nm)
        if point_is_land(candidate_lat, candidate_lon):
            continue
        coast_distance_nm = distance_to_nearest_coast_nm(candidate_lat, candidate_lon, max(min_offshore_nm + 8.0, 18.0))
        if coast_distance_nm is None:
            return candidate_lat, candidate_lon, spawn_range_nm, spawn_bearing
        if fallback is None:
            fallback = (candidate_lat, candidate_lon, spawn_range_nm, spawn_bearing)
        if coast_distance_nm >= min_offshore_nm:
            return candidate_lat, candidate_lon, spawn_range_nm, spawn_bearing
    return fallback


def random_global_latlon(lat_limit=CIVILIAN_TRAFFIC_GLOBAL_LAT_LIMIT):
    min_sin = math.sin(math.radians(-lat_limit))
    max_sin = math.sin(math.radians(lat_limit))
    lat_value = math.degrees(math.asin(random.uniform(min_sin, max_sin)))
    lon_value = random.uniform(-180.0, 180.0)
    return lat_value, lon_value


def random_global_sea_point(min_offshore_nm=CIVILIAN_TRAFFIC_MIN_OFFSHORE_NM, attempts=260):
    fallback = None
    for _ in range(attempts):
        candidate_lat, candidate_lon = random_global_latlon()
        if point_is_land(candidate_lat, candidate_lon):
            continue
        coast_distance_nm = distance_to_nearest_coast_nm(candidate_lat, candidate_lon, max(min_offshore_nm + 8.0, 18.0))
        if coast_distance_nm is None:
            return candidate_lat, candidate_lon, None, random.uniform(0, 360)
        if fallback is None:
            fallback = (candidate_lat, candidate_lon, None, random.uniform(0, 360))
        if coast_distance_nm >= min_offshore_nm:
            return candidate_lat, candidate_lon, None, random.uniform(0, 360)
    return fallback
#menu buttons







# ---------------------------------------------------------------------------
# XBT / water-column model
# ---------------------------------------------------------------------------
# XBT generates a temperature profile with depth, then derives sound speed.
# That profile is drawn in the data panel after the probe timer finishes.

class XBT:
    def __init__(self, temp_surface, temp_seabed, thermocline_depth, max_depth=2600, fallrate=2, position=None, label="XBT"):
        self.max_depth = max_depth
        self.fallrate = fallrate  # meters per second
        self.profile = []         # list of (depth, temp)
        self.sound_profile = []
        self.next_step_index = 0
        self.temp_step = 130       # interval to record temperature
        self.temp_surface = temp_surface
        self.temp_seabed = temp_seabed
        self.thermocline_depth = thermocline_depth
        self.position = pygame.Vector2(position) if position is not None else None
        self.label = label

        # precompute depth steps
        self.depth_steps = list(range(0, self.max_depth + self.temp_step, self.temp_step))

    def update(self):
        self.profile.clear()
        self.sound_profile.clear()
        self.delta_temp = self.temp_surface - self.temp_seabed

        for d in self.depth_steps:
        # compute temperature
            
            temp = self.temp_surface - self.delta_temp / (1 + np.exp(-0.012 * (d - self.thermocline_depth))) - 0.00111 * d

                
            
               

            self.profile.append((d, temp))
            self.next_step_index += 1

        for d , t in self.profile:
            

            S = 35

            c = (1448.96
            + 4.591*t
            - 5.304e-2*t**2
            + 2.374e-4*t**3
            + 1.340*(S-35)
            + 1.630e-2*d
            + 1.675e-7*d**2
            - 1.025e-2*t*(S-35)
            - 7.139e-13*t*d**3)
        
        
            self.sound_profile.append((d, c))





def sound_speed(T, D, S=35):
    """
    T: temperature in Celsius
    S: salinity in PSU
    D: depth in meters (pressure effect)
    Returns sound speed in m/s
    """

    c = (1448.96
         + 4.591*T
         - 5.304e-2*T**2
         + 2.374e-4*T**3
         + 1.340*(S-35)
         + 1.630e-2*D
         + 1.675e-7*D**2
         - 1.025e-2*T*(S-35)
         - 7.139e-13*T*D**3)
    return c


def current_sound_profile():
    if latest_xbt_profile is not None and getattr(latest_xbt_profile, "sound_profile", None):
        return latest_xbt_profile.sound_profile
    if "default_environment_xbt" in globals() and getattr(default_environment_xbt, "sound_profile", None):
        return default_environment_xbt.sound_profile
    return []


def sound_speed_at_depth(depth_m):
    profile = current_sound_profile()
    if not profile:
        return 1500.0
    depth_m = max(0.0, min(float(depth_m), profile[-1][0]))
    for index in range(1, len(profile)):
        d0, c0 = profile[index - 1]
        d1, c1 = profile[index]
        if d0 <= depth_m <= d1:
            frac = (depth_m - d0) / max(1.0, d1 - d0)
            return c0 + (c1 - c0) * frac
    return profile[-1][1]


def acoustic_layer_depth():
    profile = current_sound_profile()
    if len(profile) < 3:
        return thermocline_depth
    strongest_gradient = None
    layer_depth = thermocline_depth
    for index in range(1, len(profile)):
        d0, c0 = profile[index - 1]
        d1, c1 = profile[index]
        gradient = abs((c1 - c0) / max(1.0, d1 - d0))
        if strongest_gradient is None or gradient > strongest_gradient:
            strongest_gradient = gradient
            layer_depth = (d0 + d1) * 0.5
    return layer_depth


def thermocline_crosses(source_depth_m, receiver_depth_m):
    layer = acoustic_layer_depth()
    return (source_depth_m - layer) * (receiver_depth_m - layer) < 0



def deep_sound_channel_axis_depth():
    profile = current_sound_profile()
    if not profile:
        return max(250.0, min(seabed_depth * 0.55, 1200.0))
    lower_profile = [(depth, speed) for depth, speed in profile if depth >= 80.0]
    if not lower_profile:
        lower_profile = profile
    return min(lower_profile, key=lambda item: item[1])[0]


def environmental_acoustic_loss(distance_nmi, freq_hz, source_depth_m=0, receiver_depth_m=0):
    distance_nmi = max(0.0, float(distance_nmi))
    source_depth_m = max(0.0, float(source_depth_m or 0))
    receiver_depth_m = max(0.0, float(receiver_depth_m or 0))
    freq_khz = max(0.05, float(freq_hz) / 1000.0)
    extra_loss = 0.0

    if thermocline_crosses(source_depth_m, receiver_depth_m):
        extra_loss += min(18.0, 7.0 + 2.2 * distance_nmi + 0.9 * freq_khz)

    layer_depth = acoustic_layer_depth()
    if abs(source_depth_m - layer_depth) <= 80 or abs(receiver_depth_m - layer_depth) <= 80:
        extra_loss += 2.5

    axis_depth = deep_sound_channel_axis_depth()
    in_deep_channel = (
        distance_nmi >= 4.0 and
        freq_khz <= 1.5 and
        abs(source_depth_m - axis_depth) <= 260.0 and
        abs(receiver_depth_m - axis_depth) <= 360.0
    )
    if in_deep_channel:
        extra_loss -= min(12.0, 3.0 + 0.38 * distance_nmi)

    same_surface_duct = (
        distance_nmi >= 2.0 and
        source_depth_m <= max(80.0, layer_depth * 0.55) and
        receiver_depth_m <= max(80.0, layer_depth * 0.55)
    )
    if same_surface_duct:
        extra_loss -= min(6.0, 1.5 + 0.18 * distance_nmi)

    shadow_zone = (
        distance_nmi >= 2.0 and
        thermocline_crosses(source_depth_m, receiver_depth_m) and
        min(abs(source_depth_m - layer_depth), abs(receiver_depth_m - layer_depth)) > 70.0
    )
    if shadow_zone:
        extra_loss += min(14.0, 3.0 + 1.1 * distance_nmi + 0.45 * freq_khz)

    near_bottom = (
        source_depth_m >= seabed_depth - 180 or
        receiver_depth_m >= seabed_depth - 180
    )
    if near_bottom:
        extra_loss += min(10.0, 3.0 + 1.2 * distance_nmi)

    return extra_loss


def bottom_clutter_db(distance_nmi, sonar_depth_m, freq_hz):
    bottom_gap = max(0.0, seabed_depth - float(sonar_depth_m or 0))
    if bottom_gap > 1400:
        bottom_factor = 0.35
    else:
        bottom_factor = 1.0 - min(0.65, bottom_gap / 2200)
    range_loss = 9.0 * math.log10(max(1.0, distance_nmi * 1852.0))
    freq_bonus = min(8.0, max(0.0, (freq_hz - 1200.0) / 450.0))
    return 88.0 + freq_bonus + (18.0 * bottom_factor) - range_loss


# ---------------------------------------------------------------------------
# Navigation/acoustic geometry
# ---------------------------------------------------------------------------
def haversine(lat1_d, lon1_d, lat2_d, lon2_d):
    """
    Calculate great-circle distance (in nautical miles) and initial bearing (in degrees)
    between two points given in decimal degrees.
    """
    # Convert degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1_d, lon1_d, lat2_d, lon2_d])
    
    # Differences
    dlat = lat2 - lat1
    dlon = lon2 - lon1

    # Haversine formula for distance
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    # Earth radius in nautical miles
    R = 3440.065
    distance = R * c

    # Bearing calculation
  

    return distance

def haversine_bearing(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
    initial_bearing = math.atan2(x, y)
    
    # Convert from radians to degrees and normalize
    bearing = (math.degrees(initial_bearing) + 360) % 360
    return bearing


channel_array = []


class Channel:
    def __init__(self, channel_number):
        self.channel_number = channel_number



        
channel_array = [Channel(i) for i in range(1, 100)]
channel_names = [f"{ch.channel_number}" for ch in channel_array]

sono_selection = "SSQ-53D(DIFAR)"
sono_channel = '1'
displayed_channel = 1
auto_buoy_enabled = False
auto_buoy_pending_drops = []
auto_buoy_trigger_nm = 0.5
auto_buoy_last_drop_time = 0.0
auto_buoy_min_drop_interval = 0.8


# Create a panel to group all controls (optional but helps alignment)
controls_panel = pygame_gui.elements.UIPanel(
    relative_rect=pygame.Rect((10, 1002), (940, 68)),  # compact row around labels/buttons
    manager=manager
)

# Labels above elements
launch_label = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((20, 4), (150, 20)),
    text="Launch",
    manager=manager,
    container=controls_panel
)

arm_label = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((180, 4), (150, 20)),
    text="Arm",
    manager=manager,
    container=controls_panel
)

depth_label = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((340, 4), (50, 20)),
    text="Depth",
    manager=manager,
    container=controls_panel
)

sonobuoy_label = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((400, 4), (200, 20)),
    text="Sonobuoy",
    manager=manager,
    container=controls_panel
)

channel_label = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((610, 4), (100, 20)),
    text="Channel",
    manager=manager,
    container=controls_panel
)

torpedo_freq_label = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((830, 4), (100, 20)),
    text="Freq",
    manager=manager,
    container=controls_panel
)

# Buttons and dropdowns below labels
launch_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((20, 26), (150, 30)),
    text='LAUNCH',
    manager=manager,
    container=controls_panel,
    tool_tip_text="Launch your selected equipment."
)

arm_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((180, 26), (150, 30)),
    text="ARM", 
    manager=manager,
    container=controls_panel,
    tool_tip_text="Arm your selected equipment."
)
auto_buoy_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((720, 26), (100, 30)),
    text="AUTO",
    manager=manager,
    container=controls_panel,
    tool_tip_text="Toggle auto buoy drops from the active search pattern."
)
auto_buoy_status_label = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((824, 31), (96, 20)),
    text="OFF",
    manager=manager,
    container=controls_panel
)


def sync_arm_button_style():
    new_colour = pygame.Color("#99C979") if sonoArmed else pygame.Color("#b13b3b")
    arm_button.colours["normal_bg"] = new_colour
    arm_button.colours["hovered_bg"] = new_colour
    arm_button.colours["active_bg"] = new_colour
    arm_button.rebuild()


def sync_auto_buoy_button_style():
    new_colour = pygame.Color("#99C979") if auto_buoy_enabled else pygame.Color("#b13b3b")
    auto_buoy_button.set_text("AUTO ON" if auto_buoy_enabled else "AUTO")
    auto_buoy_button.colours["normal_bg"] = new_colour
    auto_buoy_button.colours["hovered_bg"] = new_colour
    auto_buoy_button.colours["active_bg"] = new_colour
    auto_buoy_button.rebuild()
    remaining = sum(1 for drop in auto_buoy_pending_drops if not drop.get("dropped"))
    if auto_buoy_enabled:
        auto_buoy_status_label.set_text(f"{remaining} @ {auto_buoy_trigger_nm:.1f}NM")
    else:
        auto_buoy_status_label.set_text("OFF")


def sync_ship_inject_button_style():
    new_colour = pygame.Color("#99C979") if ship_injection_enabled else pygame.Color("#b13b3b")
    prefix = "XPL " if xplane == 1 else ""
    ship_inject_button.set_text(f"{prefix}SHIPS ON" if ship_injection_enabled else f"{prefix}SHIPS OFF")
    ship_inject_button.colours["normal_bg"] = new_colour
    ship_inject_button.colours["hovered_bg"] = new_colour
    ship_inject_button.colours["active_bg"] = new_colour
    ship_inject_button.rebuild()


def multiplayer_is_host_role(role=None):
    return (role or multiplayer_role) in ("HOST", "SERVER")


def multiplayer_contact_password_ok(password=None):
    candidate = multiplayer_contact_password if password is None else password
    return str(candidate or "") == MULTIPLAYER_CONTACT_PASSWORD


PROTECTED_CONTACT_COMMANDS = {"delete"}


def multiplayer_host_requires_password():
    return bool(multiplayer_host_seen and multiplayer_host_seen.get("password_required"))


def multiplayer_can_join_server():
    return True

def sanitize_multiplayer_callsign(text):
    clean = re.sub(r"[^A-Za-z0-9_-]", "", str(text or "").strip().upper())
    return (clean or "AIRCRAFT")[:12]


def dropdown_value(value):
    if isinstance(value, (list, tuple)) and value:
        return str(value[0])
    return str(value)


def set_registered_ui_base_rect(element, base_rect):
    for index, (registered, _) in enumerate(ui_elements):
        if registered is element:
            ui_elements[index] = (registered, base_rect.copy())
            break
    else:
        register_ui_element(element, base_rect.copy())
    scaled_rect = internal_rect_to_screen_rect(base_rect)
    element.set_relative_position((scaled_rect.x, scaled_rect.y))
    element.set_dimensions((scaled_rect.w, scaled_rect.h))


def multiplayer_platform_selector_rect():
    return pygame.Rect((390, 705), (300, 30))


def control_contact_allowed(contact, player_type=None):
    player_type = dropdown_value(player_type or MULTIPLAYER_PLAYER_TYPE)
    internal_type = getattr(contact, "internal_type", "")
    if player_type == "Ship":
        return internal_type == "Surface-Ship"
    if player_type == "Submarine":
        return internal_type == "Sub-surface"
    return False


def control_contact_label(contact):
    return f"{int(getattr(contact, 'track_number', 0) or 0):04d} {getattr(contact, 'name', 'Contact')} {getattr(contact, 'internal_class', '')}".strip()


def control_contact_options_for_type(player_type=None):
    options = ["Auto"]
    if "contacts" not in globals():
        return options
    for contact in contacts:
        if control_contact_allowed(contact, player_type):
            label = control_contact_label(contact)
            if label not in options:
                options.append(label)
    return options


def selected_control_contact_track_from_label(label):
    match = re.match(r"(\d+)", dropdown_value(label).strip())
    return int(match.group(1)) if match else None


def refresh_control_contact_dropdown(selected=None):
    global multiplayer_contact_dropdown, ownship_control_contact_key, ownship_control_contact_options
    if "multiplayer_contact_dropdown" not in globals():
        return
    selected = dropdown_value(selected if selected is not None else ownship_control_contact_key)
    options = control_contact_options_for_type()
    selected_track = selected_control_contact_track_from_label(selected)
    if selected not in options and selected_track is None:
        selected_track = ownship_control_contact_track
    if selected not in options and selected_track is not None:
        track_prefix = f"{int(selected_track):04d} "
        selected = next((option for option in options if option.startswith(track_prefix)), selected)
    if selected not in options:
        selected = "Auto"
    current_selected = dropdown_value(getattr(multiplayer_contact_dropdown, "selected_option", selected))
    if options == ownship_control_contact_options and current_selected == selected:
        return
    visible = getattr(multiplayer_contact_dropdown, "visible", False)
    base_rect = multiplayer_platform_selector_rect()
    rect = internal_rect_to_screen_rect(base_rect)
    ui_elements[:] = [item for item in ui_elements if item[0] is not multiplayer_contact_dropdown]
    multiplayer_contact_dropdown.kill()
    ownship_control_contact_options = options
    ownship_control_contact_key = selected
    multiplayer_contact_dropdown = force_dropdown_down(pygame_gui.elements.UIDropDownMenu(
        options_list=options,
        starting_option=selected,
        relative_rect=rect,
        manager=manager
    ))
    register_ui_element(multiplayer_contact_dropdown, base_rect)
    if visible:
        multiplayer_contact_dropdown.show()
    else:
        multiplayer_contact_dropdown.hide()


def update_multiplayer_platform_selector_visibility():
    menu_visible = bool(getattr(contact_panel, "visible", False))
    set_registered_ui_base_rect(multiplayer_aircraft_dropdown, multiplayer_platform_selector_rect())
    set_registered_ui_base_rect(multiplayer_contact_dropdown, multiplayer_platform_selector_rect())
    if player_is_ship_or_sub():
        multiplayer_aircraft_dropdown.hide()
        if menu_visible:
            multiplayer_contact_dropdown.show()
        else:
            multiplayer_contact_dropdown.hide()
    else:
        multiplayer_contact_dropdown.hide()
        if menu_visible:
            multiplayer_aircraft_dropdown.show()
        else:
            multiplayer_aircraft_dropdown.hide()


def selected_ownship_control_contact():
    global ownship_control_contact_track, ownship_control_contact_key
    if "contacts" not in globals() or not player_is_ship_or_sub():
        return None
    selected_track = selected_control_contact_track_from_label(ownship_control_contact_key)
    if selected_track is not None:
        for contact in contacts:
            if int(getattr(contact, "track_number", -1) or -1) == selected_track and control_contact_allowed(contact):
                ownship_control_contact_track = selected_track
                return contact
    for contact in contacts:
        if control_contact_allowed(contact):
            ownship_control_contact_track = int(getattr(contact, "track_number", 0) or 0)
            return contact
    ownship_control_contact_track = None
    return None


def update_multiplayer_settings_from_menu():
    global MULTIPLAYER_CALLSIGN, MULTIPLAYER_AIRCRAFT_TYPE, MULTIPLAYER_PLAYER_TYPE, MULTIPLAYER_TEAM, ownship_control_contact_key, multiplayer_contact_password
    MULTIPLAYER_CALLSIGN = sanitize_multiplayer_callsign(multiplayer_callsign_entry.get_text())
    if multiplayer_callsign_entry.get_text() != MULTIPLAYER_CALLSIGN:
        multiplayer_callsign_entry.set_text(MULTIPLAYER_CALLSIGN)
    MULTIPLAYER_AIRCRAFT_TYPE = dropdown_value(getattr(multiplayer_aircraft_dropdown, "selected_option", MULTIPLAYER_AIRCRAFT_TYPE))
    MULTIPLAYER_PLAYER_TYPE = dropdown_value(getattr(multiplayer_player_type_dropdown, "selected_option", MULTIPLAYER_PLAYER_TYPE))
    MULTIPLAYER_TEAM = dropdown_value(getattr(multiplayer_team_dropdown, "selected_option", MULTIPLAYER_TEAM))
    multiplayer_contact_password = multiplayer_password_entry.get_text().strip()
    ownship_control_contact_key = dropdown_value(getattr(multiplayer_contact_dropdown, "selected_option", ownship_control_contact_key))


def multiplayer_platform_label():
    player_type = dropdown_value(MULTIPLAYER_PLAYER_TYPE)
    return dropdown_value(MULTIPLAYER_AIRCRAFT_TYPE) if player_type == "Aircraft" else player_type


def multiplayer_password_status_label_text():
    if not multiplayer_contact_password:
        return "PW NONE"
    return "PW OK" if multiplayer_contact_password_ok() else "PW BAD"


def sync_multiplayer_menu_status():
    player_type = dropdown_value(MULTIPLAYER_PLAYER_TYPE)
    platform = multiplayer_platform_label()
    team = dropdown_value(MULTIPLAYER_TEAM) if player_type != "Aircraft" else ""
    password_suffix = f" / {multiplayer_password_status_label_text()}" if multiplayer_role in ("SERVER", "HOST", "JOIN") else ""
    suffix = f" / {platform}" + (f" / {team}" if team else "") + password_suffix
    if player_type in ("Ship", "Submarine"):
        suffix += f" / {dropdown_value(ownship_control_contact_key)}"
    if multiplayer_role == "OFF":
        multiplayer_status_label.set_text("MP OFF" + suffix)
    elif multiplayer_role == "SERVER":
        multiplayer_status_label.set_text(f"SERVER {MULTIPLAYER_CALLSIGN}{suffix}")
    elif multiplayer_role == "HOST":
        multiplayer_status_label.set_text(f"HOSTING {MULTIPLAYER_CALLSIGN}{suffix}")
    elif multiplayer_host_seen:
        age = max(0.0, time.time() - multiplayer_host_seen.get("last_seen", time.time()))
        multiplayer_status_label.set_text(f"JOIN READY: host {multiplayer_host_seen.get('callsign', 'HOST')} ({age:.0f}s)")
    else:
        multiplayer_status_label.set_text("JOIN: waiting for host")

def set_multiplayer_role(role):
    global multiplayer_role, multiplayer_enabled, multiplayer_host_seen
    update_multiplayer_settings_from_menu()
    multiplayer_role = role if role in ("OFF", "SERVER", "HOST", "JOIN") else "OFF"
    multiplayer_enabled = multiplayer_role != "OFF"
    multiplayer_peers.clear()
    if multiplayer_role == "OFF":
        multiplayer_host_seen = None
        close_multiplayer_socket()
        print("[MP] multiplayer disabled")
    else:
        if multiplayer_role == "JOIN":
            multiplayer_host_seen = None
        ensure_multiplayer_socket()
        label = "server" if multiplayer_role == "SERVER" else multiplayer_role.lower()
        print(f"[MP] {label} mode as {MULTIPLAYER_CALLSIGN} ({MULTIPLAYER_AIRCRAFT_TYPE})")
    if "update_multiplayer_channel_assignments" in globals():
        update_multiplayer_channel_assignments()
    sync_multiplayer_menu_status()


def sync_bearing_lines_button_style():
    new_colour = pygame.Color("#99C979") if bearing_lines_visible else pygame.Color("#b13b3b")
    bearing_lines_button.set_text("LINES ON" if bearing_lines_visible else "LINES OFF")
    bearing_lines_button.colours["normal_bg"] = new_colour
    bearing_lines_button.colours["hovered_bg"] = new_colour
    bearing_lines_button.colours["active_bg"] = new_colour
    bearing_lines_button.rebuild()


depth_dropdown = force_dropdown_up(pygame_gui.elements.UIDropDownMenu(
    options_list=["90","400","1000"],
    starting_option="90", 
    relative_rect=pygame.Rect((350, 1028), (50, 30)),
    manager=manager,
    expansion_height_limit=140
), 140)

sonobuoy_dropdown = force_dropdown_up(pygame_gui.elements.UIDropDownMenu(
    options_list=["SSQ-36B(XBT)","SSQ-53D(DIFAR)","STINGRAY(TORPEDO)","SSQ-62(DICASS)"], 
    starting_option="SSQ-53D(DIFAR)", 
    relative_rect=pygame.Rect((410, 1028), (200, 30)),
    manager=manager,
    expansion_height_limit=180
), 180)

current_channel_label = pygame_gui.elements.UITextBox(
    html_text=f"{sono_channel}",
    relative_rect=pygame.Rect(610, 26, 100,30),
    manager=manager,
    container=controls_panel
)
channel_range_label = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((610, 54), (320, 14)),
    text="Range: 1-99",
    manager=manager,
    container=controls_panel
)

torpedo_mode_label = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((720, 4), (90, 20)),
    text="Mode",
    manager=manager,
    container=controls_panel
)
torpedo_mode_dropdown = force_dropdown_up(pygame_gui.elements.UIDropDownMenu(
    options_list=["PASSIVE", "ACTIVE"],
    starting_option="PASSIVE",
    relative_rect=pygame.Rect((730, 1028), (100, 30)),
    manager=manager,
    expansion_height_limit=100
), 100)
selected_torpedo_mode = "PASSIVE"

torpedo_frequency_options = ["250", "300", "350", "500", "1000", "1500", "1700", "2000", "2500", "2800", "3000"]
torpedo_frequency_dropdown = force_dropdown_up(pygame_gui.elements.UIDropDownMenu(
    options_list=torpedo_frequency_options,
    starting_option="500",
    relative_rect=pygame.Rect((840, 1028), (100, 30)),
    manager=manager,
    expansion_height_limit=220
), 220)
selected_torpedo_frequency = 500.0


def sync_torpedo_control_visibility():
    torpedo_selected = sono_selection == "STINGRAY(TORPEDO)"
    passive_selected = selected_torpedo_mode == "PASSIVE"

    if torpedo_selected:
        torpedo_mode_label.show()
        torpedo_mode_dropdown.show()
        auto_buoy_button.hide()
        auto_buoy_status_label.hide()
    else:
        torpedo_mode_label.hide()
        torpedo_mode_dropdown.hide()
        if not in_menu:
            auto_buoy_button.show()
            auto_buoy_status_label.show()

    if torpedo_selected and passive_selected:
        torpedo_freq_label.show()
        torpedo_frequency_dropdown.show()
    else:
        torpedo_freq_label.hide()
        torpedo_frequency_dropdown.hide()


display_mode = "MAP"
radar_orientation = "TRACK"
radar_range_options = [0.5, 2, 5, 10, 20, 30, 60, 120]
radar_range_index = 6


def step_radar_range(direction):
    global radar_range_index
    radar_range_index = (radar_range_index + direction) % len(radar_range_options)
    radar_range = radar_range_options[radar_range_index]
    radar_range_button.set_text(f"{radar_range:g} NM")
    radar_terrain_cache["key"] = None
    return radar_range
bearing_lines_visible = True

map_mode_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((970, 10), (90, 28)),
    text="MAP",
    manager=manager
)
radar_mode_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((1065, 10), (90, 28)),
    text="RADAR",
    manager=manager
)
nav_mode_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((1160, 10), (90, 28)),
    text="NAV",
    manager=manager
)
radar_orientation_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((1160, 10), (120, 28)),
    text="TRACK UP",
    manager=manager
)
radar_range_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((1285, 10), (110, 28)),
    text="60 NM",
    manager=manager
)
bearing_lines_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((1400, 10), (110, 28)),
    text="LINES ON",
    manager=manager,
    tool_tip_text="Toggle all passive sonobuoy bearing lines and uncertainty wedges."
)
ship_inject_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((1515, 10), (110, 28)),
    text="SHIPS OFF",
    manager=manager,
    tool_tip_text="MSFS: inject GAIST ships. X-Plane: export ships and push multiplayer positions."
)

nav_heading_label = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((990, 138), (120, 24)),
    text="CMD HDG",
    manager=manager
)
nav_heading_entry = pygame_gui.elements.UITextEntryLine(
    relative_rect=pygame.Rect((1115, 136), (105, 30)),
    manager=manager
)
nav_heading_entry.set_text("120")
nav_speed_label = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((1235, 138), (120, 24)),
    text="CMD SPD",
    manager=manager
)
nav_speed_entry = pygame_gui.elements.UITextEntryLine(
    relative_rect=pygame.Rect((1360, 136), (105, 30)),
    manager=manager
)
nav_speed_entry.set_text("0")
nav_depth_label = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((1480, 138), (120, 24)),
    text="CMD DEPTH",
    manager=manager
)
nav_depth_entry = pygame_gui.elements.UITextEntryLine(
    relative_rect=pygame.Rect((1605, 136), (105, 30)),
    manager=manager
)
nav_depth_entry.set_text("0")
nav_route_entry = pygame_gui.elements.UITextEntryLine(
    relative_rect=pygame.Rect((990, 184), (610, 30)),
    manager=manager
)
nav_route_entry.set_text("")
nav_import_route_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((1610, 184), (130, 30)),
    text="IMPORT ROUTE",
    manager=manager
)
nav_route_status_label = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((990, 222), (750, 26)),
    text="No route",
    manager=manager
)
nav_elements = [
    nav_heading_label,
    nav_heading_entry,
    nav_speed_label,
    nav_speed_entry,
    nav_depth_label,
    nav_depth_entry,
    nav_route_entry,
    nav_import_route_button,
    nav_route_status_label
]
for element in nav_elements:
    element.hide()


# === REGISTER LABELS ===
register_ui_element(launch_label, pygame.Rect((20, 4), (150, 20)))
register_ui_element(arm_label, pygame.Rect((180, 4), (150, 20)))
register_ui_element(depth_label, pygame.Rect((340, 4), (50, 20)))
register_ui_element(sonobuoy_label, pygame.Rect((400, 4), (200, 20)))
register_ui_element(channel_label, pygame.Rect((610, 4), (100, 20)))
register_ui_element(channel_range_label, pygame.Rect((610, 54), (320, 14)))
register_ui_element(torpedo_mode_label, pygame.Rect((720, 4), (90, 20)))
register_ui_element(torpedo_freq_label, pygame.Rect((830, 4), (100, 20)))

# === REGISTER BUTTONS ===
register_ui_element(launch_button, pygame.Rect((20, 26), (150, 30)))
register_ui_element(arm_button, pygame.Rect((180, 26), (150, 30)))
register_ui_element(auto_buoy_button, pygame.Rect((720, 26), (100, 30)))
register_ui_element(auto_buoy_status_label, pygame.Rect((824, 31), (96, 20)))

# === REGISTER DROPDOWNS ===
register_ui_element(depth_dropdown, pygame.Rect((350, 1028), (50, 30)))
register_ui_element(sonobuoy_dropdown, pygame.Rect((410, 1028), (200, 30)))
register_ui_element(torpedo_mode_dropdown, pygame.Rect((730, 1028), (100, 30)))
register_ui_element(torpedo_frequency_dropdown, pygame.Rect((840, 1028), (100, 30)))
register_ui_element(map_mode_button, map_mode_button.relative_rect)
register_ui_element(radar_mode_button, radar_mode_button.relative_rect)
register_ui_element(nav_mode_button, nav_mode_button.relative_rect)
register_ui_element(radar_orientation_button, radar_orientation_button.relative_rect)
register_ui_element(radar_range_button, radar_range_button.relative_rect)
register_ui_element(bearing_lines_button, bearing_lines_button.relative_rect)
register_ui_element(ship_inject_button, ship_inject_button.relative_rect)
register_ui_element(nav_heading_label, nav_heading_label.relative_rect)
register_ui_element(nav_heading_entry, nav_heading_entry.relative_rect)
register_ui_element(nav_speed_label, nav_speed_label.relative_rect)
register_ui_element(nav_speed_entry, nav_speed_entry.relative_rect)
register_ui_element(nav_depth_label, nav_depth_label.relative_rect)
register_ui_element(nav_depth_entry, nav_depth_entry.relative_rect)
register_ui_element(nav_route_entry, nav_route_entry.relative_rect)
register_ui_element(nav_import_route_button, nav_import_route_button.relative_rect)
register_ui_element(nav_route_status_label, nav_route_status_label.relative_rect)
layout_top_mode_buttons(screen_width, screen_height)
sync_stateful_button_styles()
sync_bearing_lines_button_style()
sync_arm_button_style()
sync_auto_buoy_button_style()
sync_ship_inject_button_style()
sync_multiplayer_menu_status()
sync_torpedo_control_visibility()


search_pattern_anchor_world = None
search_pattern_waypoints = []
search_pattern_buoy_points = []
search_pattern_saved_references = []
search_pattern_output_string = ""
search_pattern_selected_type = "Parallel Sweep"
search_pattern_anchor_mode = "START"

search_pattern_panel = pygame_gui.elements.UIPanel(
    relative_rect=pygame.Rect((1048, 54), (840, 386)),
    manager=manager
)
search_pattern_title = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((12, 8), (250, 24)),
    text="Search Pattern",
    manager=manager,
    container=search_pattern_panel
)
search_pattern_close_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((764, 8), (56, 26)),
    text="CLOSE",
    manager=manager,
    container=search_pattern_panel
)
search_pattern_type_label = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((12, 44), (82, 22)),
    text="Pattern",
    manager=manager,
    container=search_pattern_panel
)
search_pattern_type_dropdown = pygame_gui.elements.UIDropDownMenu(
    options_list=["Parallel Sweep"],
    starting_option="Parallel Sweep",
    relative_rect=pygame.Rect((96, 42), (170, 28)),
    manager=manager,
    container=search_pattern_panel
)
search_pattern_heading_label = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((284, 44), (78, 22)),
    text="Heading",
    manager=manager,
    container=search_pattern_panel
)
search_pattern_heading_entry = pygame_gui.elements.UITextEntryLine(
    relative_rect=pygame.Rect((366, 42), (70, 28)),
    manager=manager,
    container=search_pattern_panel
)
search_pattern_heading_entry.set_text("000")
search_pattern_length_label = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((454, 44), (78, 22)),
    text="Leg NM",
    manager=manager,
    container=search_pattern_panel
)
search_pattern_length_entry = pygame_gui.elements.UITextEntryLine(
    relative_rect=pygame.Rect((532, 42), (70, 28)),
    manager=manager,
    container=search_pattern_panel
)
search_pattern_length_entry.set_text("12")
search_pattern_spacing_label = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((620, 44), (68, 22)),
    text="Spacing",
    manager=manager,
    container=search_pattern_panel
)
search_pattern_spacing_entry = pygame_gui.elements.UITextEntryLine(
    relative_rect=pygame.Rect((690, 42), (70, 28)),
    manager=manager,
    container=search_pattern_panel
)
search_pattern_spacing_entry.set_text("2")
search_pattern_count_label = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((12, 82), (82, 22)),
    text="Tracks",
    manager=manager,
    container=search_pattern_panel
)
search_pattern_count_entry = pygame_gui.elements.UITextEntryLine(
    relative_rect=pygame.Rect((96, 80), (70, 28)),
    manager=manager,
    container=search_pattern_panel
)
search_pattern_count_entry.set_text("6")
search_pattern_anchor_mode_label = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((500, 82), (92, 22)),
    text="Point",
    manager=manager,
    container=search_pattern_panel
)
search_pattern_anchor_mode_dropdown = pygame_gui.elements.UIDropDownMenu(
    options_list=["START", "CENTER"],
    starting_option=search_pattern_anchor_mode,
    relative_rect=pygame.Rect((598, 80), (112, 28)),
    manager=manager,
    container=search_pattern_panel
)
search_pattern_generate_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((184, 80), (110, 28)),
    text="GENERATE",
    manager=manager,
    container=search_pattern_panel
)
search_pattern_copy_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((304, 80), (84, 28)),
    text="COPY",
    manager=manager,
    container=search_pattern_panel
)
search_pattern_import_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((398, 80), (84, 28)),
    text="IMPORT",
    manager=manager,
    container=search_pattern_panel
)
search_pattern_status_label = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((716, 82), (104, 22)),
    text="Both mouse buttons on map to place.",
    manager=manager,
    container=search_pattern_panel
)
search_pattern_datum_lat_label = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((12, 122), (70, 22)),
    text="Lat",
    manager=manager,
    container=search_pattern_panel
)
search_pattern_datum_lat_entry = pygame_gui.elements.UITextEntryLine(
    relative_rect=pygame.Rect((82, 120), (116, 28)),
    manager=manager,
    container=search_pattern_panel
)
search_pattern_datum_lon_label = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((206, 122), (70, 22)),
    text="Lon",
    manager=manager,
    container=search_pattern_panel
)
search_pattern_datum_lon_entry = pygame_gui.elements.UITextEntryLine(
    relative_rect=pygame.Rect((276, 120), (116, 28)),
    manager=manager,
    container=search_pattern_panel
)
search_pattern_apply_datum_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((402, 120), (74, 28)),
    text="SET",
    manager=manager,
    container=search_pattern_panel
)
search_pattern_offset_bearing_label = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((492, 122), (54, 22)),
    text="BRG",
    manager=manager,
    container=search_pattern_panel
)
search_pattern_offset_bearing_entry = pygame_gui.elements.UITextEntryLine(
    relative_rect=pygame.Rect((548, 120), (58, 28)),
    manager=manager,
    container=search_pattern_panel
)
search_pattern_offset_bearing_entry.set_text("000")
search_pattern_offset_range_label = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((616, 122), (42, 22)),
    text="NM",
    manager=manager,
    container=search_pattern_panel
)
search_pattern_offset_range_entry = pygame_gui.elements.UITextEntryLine(
    relative_rect=pygame.Rect((658, 120), (58, 28)),
    manager=manager,
    container=search_pattern_panel
)
search_pattern_offset_range_entry.set_text("0.0")
search_pattern_apply_offset_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((726, 120), (94, 28)),
    text="OFFSET",
    manager=manager,
    container=search_pattern_panel
)
search_pattern_import_label = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((12, 166), (92, 22)),
    text="Import",
    manager=manager,
    container=search_pattern_panel
)
search_pattern_import_entry = pygame_gui.elements.UITextEntryLine(
    relative_rect=pygame.Rect((104, 164), (716, 30)),
    manager=manager,
    container=search_pattern_panel
)
search_pattern_output_label = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((12, 210), (92, 22)),
    text="Waypoints",
    manager=manager,
    container=search_pattern_panel
)
search_pattern_output_entry = pygame_gui.elements.UITextEntryLine(
    relative_rect=pygame.Rect((104, 208), (716, 30)),
    manager=manager,
    container=search_pattern_panel
)
search_pattern_save_reference_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((104, 250), (118, 28)),
    text="SAVE REF",
    manager=manager,
    container=search_pattern_panel
)
search_pattern_clear_reference_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((232, 250), (118, 28)),
    text="CLEAR REF",
    manager=manager,
    container=search_pattern_panel
)
search_pattern_buoy_spacing_label = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((370, 252), (78, 22)),
    text="Buoy NM",
    manager=manager,
    container=search_pattern_panel
)
search_pattern_buoy_spacing_entry = pygame_gui.elements.UITextEntryLine(
    relative_rect=pygame.Rect((452, 250), (64, 28)),
    manager=manager,
    container=search_pattern_panel
)
search_pattern_buoy_spacing_entry.set_text("2.0")
search_pattern_auto_buoy_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((532, 250), (118, 28)),
    text="AUTO BUOY",
    manager=manager,
    container=search_pattern_panel
)
search_pattern_hint = pygame_gui.elements.UITextBox(
    html_text="Datum snaps to nearest 0.5 NM from aircraft. Offset is bearing/range from aircraft. Pattern is a parallel sweep.",
    relative_rect=pygame.Rect((12, 294), (808, 58)),
    manager=manager,
    container=search_pattern_panel
)

search_pattern_ui_elements = [
    search_pattern_panel,
    search_pattern_title,
    search_pattern_close_button,
    search_pattern_type_label,
    search_pattern_type_dropdown,
    search_pattern_heading_label,
    search_pattern_heading_entry,
    search_pattern_length_label,
    search_pattern_length_entry,
    search_pattern_spacing_label,
    search_pattern_spacing_entry,
    search_pattern_count_label,
    search_pattern_count_entry,
    search_pattern_anchor_mode_label,
    search_pattern_anchor_mode_dropdown,
    search_pattern_generate_button,
    search_pattern_copy_button,
    search_pattern_import_button,
    search_pattern_status_label,
    search_pattern_datum_lat_label,
    search_pattern_datum_lat_entry,
    search_pattern_datum_lon_label,
    search_pattern_datum_lon_entry,
    search_pattern_apply_datum_button,
    search_pattern_offset_bearing_label,
    search_pattern_offset_bearing_entry,
    search_pattern_offset_range_label,
    search_pattern_offset_range_entry,
    search_pattern_apply_offset_button,
    search_pattern_import_label,
    search_pattern_import_entry,
    search_pattern_output_label,
    search_pattern_output_entry,
    search_pattern_save_reference_button,
    search_pattern_clear_reference_button,
    search_pattern_buoy_spacing_label,
    search_pattern_buoy_spacing_entry,
    search_pattern_auto_buoy_button,
    search_pattern_hint
]


def set_search_pattern_panel_visible(visible):
    for element in search_pattern_ui_elements:
        if visible:
            element.show()
        else:
            element.hide()


set_search_pattern_panel_visible(False)

register_ui_element(search_pattern_panel, pygame.Rect((1048, 54), (840, 386)))
register_ui_element(search_pattern_close_button, pygame.Rect((764, 8), (56, 26)))
register_ui_element(search_pattern_type_dropdown, pygame.Rect((96, 42), (170, 28)))
register_ui_element(search_pattern_heading_entry, pygame.Rect((366, 42), (70, 28)))
register_ui_element(search_pattern_length_entry, pygame.Rect((532, 42), (70, 28)))
register_ui_element(search_pattern_spacing_entry, pygame.Rect((690, 42), (70, 28)))
register_ui_element(search_pattern_count_entry, pygame.Rect((96, 80), (70, 28)))
register_ui_element(search_pattern_anchor_mode_dropdown, pygame.Rect((598, 80), (112, 28)))
register_ui_element(search_pattern_generate_button, pygame.Rect((184, 80), (110, 28)))
register_ui_element(search_pattern_copy_button, pygame.Rect((304, 80), (84, 28)))
register_ui_element(search_pattern_import_button, pygame.Rect((398, 80), (84, 28)))
register_ui_element(search_pattern_datum_lat_entry, pygame.Rect((82, 120), (116, 28)))
register_ui_element(search_pattern_datum_lon_entry, pygame.Rect((276, 120), (116, 28)))
register_ui_element(search_pattern_apply_datum_button, pygame.Rect((402, 120), (74, 28)))
register_ui_element(search_pattern_offset_bearing_entry, pygame.Rect((548, 120), (58, 28)))
register_ui_element(search_pattern_offset_range_entry, pygame.Rect((658, 120), (58, 28)))
register_ui_element(search_pattern_apply_offset_button, pygame.Rect((726, 120), (94, 28)))
register_ui_element(search_pattern_import_entry, pygame.Rect((104, 164), (716, 30)))
register_ui_element(search_pattern_output_entry, pygame.Rect((104, 208), (716, 30)))
register_ui_element(search_pattern_save_reference_button, pygame.Rect((104, 250), (118, 28)))
register_ui_element(search_pattern_clear_reference_button, pygame.Rect((232, 250), (118, 28)))
register_ui_element(search_pattern_buoy_spacing_entry, pygame.Rect((452, 250), (64, 28)))
register_ui_element(search_pattern_auto_buoy_button, pygame.Rect((532, 250), (118, 28)))


# === REGISTER PANEL LAST ===
register_ui_element(controls_panel, pygame.Rect((10, 1002), (940, 68)))


def is_too_close_to_edge(pos, margin=edge_margin):
    x, y = pos
    return (x < margin or x > screen.get_width() - margin or
            y < margin or y > screen.get_height() - margin)

def get_new_target_coord():
    return (random.randrange(edge_margin, screen.get_width() - edge_margin),
            random.randrange(edge_margin, screen.get_height() - edge_margin))


nperseg = 0
noverlap = 0



# define the surfaces used for screens
map_surf = pygame.Surface((1920/2,1080))
data_Surface = pygame.Surface((1920/2,1080))
sono_control_surface = pygame.Surface((1920/2,300))
map_overlay_surface = pygame.Surface((1920/2,1080),pygame.SRCALPHA)
map_overlay_surface.fill((0,0,0,0))

class Timer:
    def __init__(self, delay_seconds):
        self.delay = delay_seconds
        self.start_time = None
        self.finished = False

    def start(self):
        self.start_time = time.time()
        self.finished = False

    def check(self):
        if self.start_time is None:
            return False
        if not self.finished and time.time() - self.start_time >= self.delay:
            self.finished = True
            return True
        return False

import pygame
import math

def draw_heading_line(surface, start_pos, heading_deg, length=30, color=(255, 255, 0), width=3):
    """
    Draws a line pointing in a given heading.

    Parameters:
        surface (pygame.Surface): Surface to draw on.
        start_pos (tuple): Starting point (x, y).
        heading_deg (float): Heading in degrees (0 = right/East, 90 = down/South).
        length (float): Length of the line.
        color (tuple): Line color (R,G,B).
        width (int): Line width in pixels.
        arrowhead (bool): Whether to draw an arrowhead at the end.
        arrow_size (int): Size of the arrowhead.
    """
    x0, y0 = start_pos
    heading_rad = math.radians(heading_deg)

    # Compute end point
    x1 = x0 + length * math.cos(heading_rad)
    y1 = y0 + length * math.sin(heading_rad)

    # Draw main line
    pygame.draw.line(surface, color, (x0, y0), (x1, y1), width)


contact_possible_type_list = {
    "Biological": ["Whale", "Dolphin", "Krill"],
    "Air": ["Aircraft", "Helicopter", "Unknown"],
    "Land": ["Fixed Site", "Vehicle", "Unknown"],
    "Submarine": ["Kilo", "Akula", "Delta", "Borei", "Yasen"],
    "Surface-Ship": ["Destroyer", "Frigate", "Carrier", "Civilian", "Unknown"],
    "Unknown": ["Unknown"]
}
spectrogram_marker_type_list = ["Marks Off", "Submarine", "Mil Surface", "Civ Surface", "Biological"]
spectrogram_marker_class_list = {
    "Marks Off": ["None"],
    "Submarine": ["Kilo", "Akula", "Delta IV", "Borei", "Yasen", "Emitter"],
    "Mil Surface": ["Destroyer", "Frigate", "Carrier", "Warship"],
    "Civ Surface": ["Fishing", "Yacht", "Tug", "Ferry", "Tanker", "Cargo"],
    "Biological": ["Whale", "Dolphin", "Krill"]
}
spectrogram_marker_frequencies = {
    ("Submarine", "Kilo"): [250, 1000, 1500],
    ("Submarine", "Akula"): [350, 1500, 2500],
    ("Submarine", "Delta IV"): [500, 2000, 2800],
    ("Submarine", "Borei"): [300, 1500, 2500],
    ("Submarine", "Yasen"): [500, 1700, 3000],
    ("Submarine", "Emitter"): [500, 1000, 1500],
    ("Mil Surface", "Destroyer"): [120, 245, 980, 1850, 2400],
    ("Mil Surface", "Frigate"): [105, 215, 820, 1650, 2400],
    ("Mil Surface", "Carrier"): [60, 125, 360, 720, 2400],
    ("Mil Surface", "Warship"): [100, 205, 760, 2400],
    ("Civ Surface", "Fishing"): [95, 185, 740, 2100],
    ("Civ Surface", "Yacht"): [145, 520, 1180, 2100],
    ("Civ Surface", "Tug"): [80, 160, 620, 2100],
    ("Civ Surface", "Ferry"): [115, 235, 900, 2100],
    ("Civ Surface", "Tanker"): [55, 110, 410, 2100],
    ("Civ Surface", "Cargo"): [70, 140, 480, 2100],
    ("Biological", "Whale"): [95, 180, 360, 720, 1400, 2200, 3200],
    ("Biological", "Dolphin"): [],
    ("Biological", "Krill"): []
}
contact_identity_options = ["P", "U", "F", "?F", "N", "S", "H"]
contact_identity_labels = {
    "P": "Pending",
    "U": "Unknown",
    "F": "Friendly",
    "?F": "Assumed Friendly",
    "N": "Neutral",
    "S": "Suspect",
    "H": "Hostile"
}
contact_identity_colours = {
    "P": (185, 175, 70),
    "U": (95, 165, 235),
    "F": (85, 205, 245),
    "?F": (70, 225, 245),
    "N": (40, 210, 80),
    "S": (230, 95, 45),
    "H": (220, 45, 45)
}
contact_country_options = [
    "Unknown", "United States", "United Kingdom", "Russia", "China", "France", "Germany",
    "Norway", "Denmark", "Canada", "Australia", "India", "Japan", "Neutral", "Civilian"
]

selected_contact = None
torpedo_designated_contact = None
contact_context_last_text = ""
contact_context_last_title = ""
contact_status_dropdown = None
contact_country_dropdown = None
contact_context_user_closed = False
xbt_panel_visible = False
xbt_panel_selected_label = None
xbt_panel_select_dropdown = None
ray_trace_mode = "OFF"
ray_trace_source_buoy = None
ray_trace_result = None
ray_trace_status_text = "Ray trace idle."

contact_context_panel = pygame_gui.elements.UIPanel(
    relative_rect=pygame.Rect((8, 542), (930, 288)),
    manager=manager
)
contact_context_title = pygame_gui.elements.UILabel(
    relative_rect=pygame.Rect((10, 5), (220, 24)),
    text="CONTACT",
    manager=manager,
    container=contact_context_panel
)
contact_context_info = pygame_gui.elements.UITextBox(
    html_text="",
    relative_rect=pygame.Rect((10, 32), (500, 242)),
    manager=manager,
    container=contact_context_panel
)
torpedo_designate_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((530, 112), (180, 30)),
    text="DESIG TORP",
    manager=manager,
    container=contact_context_panel
)
contact_lines_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((720, 112), (180, 30)),
    text="LINES ON",
    manager=manager,
    container=contact_context_panel
)
ship_stop_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((530, 152), (120, 30)),
    text="REQ STOP",
    manager=manager,
    container=contact_context_panel
)
ship_heading_entry = pygame_gui.elements.UITextEntryLine(
    relative_rect=pygame.Rect((656, 152), (54, 30)),
    manager=manager,
    container=contact_context_panel
)
ship_heading_entry.set_allowed_characters(['0','1','2','3','4','5','6','7','8','9'])
ship_heading_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((716, 152), (86, 30)),
    text="REQ HDG",
    manager=manager,
    container=contact_context_panel
)
ship_speed_entry = pygame_gui.elements.UITextEntryLine(
    relative_rect=pygame.Rect((808, 152), (42, 30)),
    manager=manager,
    container=contact_context_panel
)
ship_speed_entry.set_allowed_characters(['0','1','2','3','4','5','6','7','8','9','.'])
ship_speed_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((856, 152), (64, 30)),
    text="REQ SPD",
    manager=manager,
    container=contact_context_panel
)
ship_deck_lock_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((530, 192), (120, 30)),
    text="LOCK DECK",
    manager=manager,
    container=contact_context_panel
)
ship_resume_route_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((530, 192), (120, 30)),
    text="RES RTE",
    manager=manager,
    container=contact_context_panel
)
ship_route_speed_entry = pygame_gui.elements.UITextEntryLine(
    relative_rect=pygame.Rect((656, 192), (54, 30)),
    manager=manager,
    container=contact_context_panel
)
ship_route_speed_entry.set_allowed_characters(['0','1','2','3','4','5','6','7','8','9','.'])
ship_route_speed_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((716, 192), (86, 30)),
    text="RTE SPD",
    manager=manager,
    container=contact_context_panel
)
ship_spawn_aft_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((808, 192), (112, 30)),
    text="SPAWN AFT",
    manager=manager,
    container=contact_context_panel
)
contact_route_entry = pygame_gui.elements.UITextEntryLine(
    relative_rect=pygame.Rect((530, 232), (260, 30)),
    manager=manager,
    container=contact_context_panel
)
contact_route_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((798, 232), (122, 30)),
    text="SET RTE",
    manager=manager,
    container=contact_context_panel
)
contact_delete_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((760, 5), (82, 26)),
    text="DELETE",
    manager=manager,
    container=contact_context_panel
)
contact_context_close_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((850, 5), (64, 26)),
    text="CLOSE",
    manager=manager,
    container=contact_context_panel
)
contact_type_dropdown = pygame_gui.elements.UIDropDownMenu(
    options_list=list(contact_possible_type_list.keys()),
    starting_option="Unknown",
    relative_rect=pygame.Rect((530, 580), (180, 30)),
    manager=manager,
    expansion_height_limit=180
)
contact_class_dropdown = pygame_gui.elements.UIDropDownMenu(
    options_list=contact_possible_type_list["Unknown"],
    starting_option="Unknown",
    relative_rect=pygame.Rect((720, 580), (205, 30)),
    manager=manager,
    expansion_height_limit=180
)
contact_status_dropdown = pygame_gui.elements.UIDropDownMenu(
    options_list=contact_identity_options,
    starting_option="P",
    relative_rect=pygame.Rect((530, 620), (180, 30)),
    manager=manager,
    expansion_height_limit=180
)
contact_country_dropdown = pygame_gui.elements.UIDropDownMenu(
    options_list=contact_country_options,
    starting_option="Unknown",
    relative_rect=pygame.Rect((720, 620), (205, 30)),
    manager=manager,
    expansion_height_limit=180
)
contact_context_panel.hide()
torpedo_designate_button.hide()
contact_lines_button.hide()
ship_stop_button.hide()
ship_heading_entry.hide()
ship_heading_button.hide()
ship_speed_entry.hide()
ship_speed_button.hide()
ship_deck_lock_button.hide()
ship_resume_route_button.hide()
ship_route_speed_entry.hide()
ship_route_speed_button.hide()
ship_spawn_aft_button.hide()
contact_route_entry.hide()
contact_route_button.hide()
contact_delete_button.hide()
contact_context_close_button.hide()
contact_type_dropdown.hide()
contact_class_dropdown.hide()
contact_status_dropdown.hide()
contact_country_dropdown.hide()
register_ui_element(contact_context_panel, contact_context_panel.relative_rect)

xbt_tab_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((8, 542), (72, 26)),
    text="XBT",
    manager=manager,
    tool_tip_text="Open XBT temperature and sound-speed profile panel."
)
xbt_panel_close_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((862, 548), (64, 26)),
    text="CLOSE",
    manager=manager
)
xbt_raytrace_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((88, 548), (104, 26)),
    text="RAY POINT",
    manager=manager,
    tool_tip_text="Click a point to trace rays from the selected XBT using the selected XBT profile."
)
xbt_raytrace_clear_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect((198, 548), (70, 26)),
    text="CLEAR",
    manager=manager
)
xbt_panel_select_dropdown = pygame_gui.elements.UIDropDownMenu(
    options_list=["None"],
    starting_option="None",
    relative_rect=pygame.Rect((690, 548), (160, 26)),
    manager=manager,
    expansion_height_limit=180
)
xbt_tab_button.hide()
xbt_panel_close_button.hide()
xbt_raytrace_button.hide()
xbt_raytrace_clear_button.hide()
xbt_panel_select_dropdown.hide()
register_ui_element(xbt_tab_button, xbt_tab_button.relative_rect)
register_ui_element(xbt_panel_close_button, xbt_panel_close_button.relative_rect)
register_ui_element(xbt_raytrace_button, xbt_raytrace_button.relative_rect)
register_ui_element(xbt_raytrace_clear_button, xbt_raytrace_clear_button.relative_rect)
register_ui_element(xbt_panel_select_dropdown, xbt_panel_select_dropdown.relative_rect)

def sync_torpedo_designate_button_style():
    if selected_contact is not None and torpedo_designated_contact is selected_contact:
        torpedo_designate_button.set_text("TORP DESIG")
        new_colour = pygame.Color("#99C979")
    else:
        torpedo_designate_button.set_text("DESIG TORP")
        new_colour = pygame.Color("#4d6fa8")
    torpedo_designate_button.colours["normal_bg"] = new_colour
    torpedo_designate_button.colours["hovered_bg"] = new_colour
    torpedo_designate_button.colours["active_bg"] = new_colour
    torpedo_designate_button.rebuild()



def hide_ship_command_controls():
    ship_stop_button.hide()
    ship_heading_entry.hide()
    ship_heading_button.hide()
    ship_speed_entry.hide()
    ship_speed_button.hide()
    ship_deck_lock_button.hide()
    ship_resume_route_button.hide()
    ship_route_speed_entry.hide()
    ship_route_speed_button.hide()
    ship_spawn_aft_button.hide()
    contact_route_entry.hide()
    contact_route_button.hide()


def show_ship_command_controls(update_heading_text=False):
    ship_stop_button.show()
    ship_heading_entry.show()
    ship_heading_button.set_text("REQ HDG")
    ship_heading_button.show()
    ship_speed_entry.show()
    ship_speed_button.show()
    ship_resume_route_button.show()
    ship_route_speed_entry.show()
    ship_route_speed_button.show()
    sync_ship_command_controls(update_heading_text=update_heading_text)
    if ship_deck_carry_enabled:
        ship_deck_lock_button.show()
    else:
        ship_deck_lock_button.hide()
    if selected_contact_is_friendly_surface_ship():
        contact_route_entry.show()
        contact_route_button.show()
        ship_spawn_aft_button.hide()
    else:
        contact_route_entry.hide()
        contact_route_button.hide()
        ship_spawn_aft_button.show()

def sync_ship_command_controls(update_heading_text=False):
    if not selected_contact_is_surface_ship():
        return
    ensure_ship_command_state(selected_contact)
    ship_stop_button.set_text("REQ UNDERWAY" if ship_commanded_stopped(selected_contact) else "REQ STOP")
    deck_locked = ship_deck_lock_is_active_for_contact(selected_contact)
    ship_deck_lock_button.set_text("UNLOCK DECK" if deck_locked else "LOCK DECK")
    deck_colour = pygame.Color("#99C979") if deck_locked else pygame.Color("#4d6fa8")
    ship_deck_lock_button.colours["normal_bg"] = deck_colour
    ship_deck_lock_button.colours["hovered_bg"] = deck_colour
    ship_deck_lock_button.colours["active_bg"] = deck_colour
    ship_deck_lock_button.rebuild()
    if update_heading_text or not ship_heading_entry.get_text().strip():
        ship_heading_entry.set_text(f"{float(getattr(selected_contact, 'commanded_heading', getattr(selected_contact, 'bearing', 0)) or 0) % 360:03.0f}")
    if update_heading_text or not ship_speed_entry.get_text().strip():
        ship_speed_entry.set_text(f"{float(getattr(selected_contact, 'commanded_speed', getattr(selected_contact, 'speed', 0.0)) or 0.0):.1f}")
    if update_heading_text or not ship_route_speed_entry.get_text().strip():
        route_speed = float(getattr(selected_contact, "route_speed_kts", getattr(selected_contact, "commanded_speed", getattr(selected_contact, "speed", 0.0))) or 0.0)
        ship_route_speed_entry.set_text(f"{route_speed:.1f}")
    has_route = len(getattr(selected_contact, "route_waypoints", []) or []) > 1
    ship_resume_route_button.set_text("RES RTE" if has_route else "NO RTE")


def sync_contact_lines_button_style():
    if selected_contact is not None and getattr(selected_contact, "bearing_lines_hidden", False):
        contact_lines_button.set_text("LINES OFF")
        new_colour = pygame.Color("#b13b3b")
    else:
        contact_lines_button.set_text("LINES ON")
        new_colour = pygame.Color("#99C979")
    contact_lines_button.colours["normal_bg"] = new_colour
    contact_lines_button.colours["hovered_bg"] = new_colour
    contact_lines_button.colours["active_bg"] = new_colour
    contact_lines_button.rebuild()

def selected_contact_is_surface_ship():
    if selected_contact is None:
        return False
    return (
        getattr(selected_contact, "internal_type", "") == "Surface-Ship" or
        getattr(selected_contact, "classification_type", "") == "Surface-Ship"
    )

def selected_contact_is_friendly_surface_ship():
    if not selected_contact_is_surface_ship():
        return False
    identity = str(getattr(selected_contact, "identity_status", "P") or "P")
    team = str(getattr(selected_contact, "team", "Neutral") or "Neutral")
    return identity in ("F", "?F") or team == "BLUFOR"

def pause_ship_route_for_manual(contact, reason="manual command"):
    if getattr(contact, "route_active", False):
        contact.route_active = False
        contact.route_manual_paused = True
        contact.route_status = f"Route paused: {reason}"
        print(f"Ship track {getattr(contact, 'track_number', '?')} route paused for {reason}")


def request_ship_stop(contact):
    ensure_ship_command_state(contact)
    pause_ship_route_for_manual(contact, "speed command")
    if ship_commanded_stopped(contact):
        contact.commanded_speed = max(ship_default_underway_speed_kts, float(getattr(contact, "ship_underway_speed", ship_default_underway_speed_kts) or ship_default_underway_speed_kts))
        print(f"Ship track {getattr(contact, 'track_number', '?')} requested underway at {contact.commanded_speed:.1f} kt")
    else:
        contact.ship_underway_speed = max(ship_default_underway_speed_kts, float(getattr(contact, "speed", 0.0) or 0.0), float(getattr(contact, "commanded_speed", 0.0) or 0.0))
        contact.commanded_speed = 0.0
        current_speed = float(getattr(contact, "speed", 0.0) or 0.0)
        print(f"Ship track {getattr(contact, 'track_number', '?')} requested stop; slowing from {current_speed:.1f} kt")


def request_ship_heading(contact, heading_deg):
    try:
        heading_deg = float(heading_deg) % 360.0
    except (TypeError, ValueError):
        heading_deg = float(getattr(contact, "bearing", 0) or 0) % 360.0
    ensure_ship_command_state(contact)
    pause_ship_route_for_manual(contact, "heading command")
    current_speed = max(0.0, float(getattr(contact, "speed", 0.0) or 0.0))
    if current_speed > 0.2:
        contact.commanded_speed = current_speed
        contact.ship_underway_speed = max(
            current_speed,
            float(getattr(contact, "ship_underway_speed", 0.0) or 0.0),
            float(getattr(contact, "commanded_speed", 0.0) or 0.0)
        )
    contact.commanded_heading = heading_deg
    print(f"Ship track {getattr(contact, 'track_number', '?')} requested heading {heading_deg:03.0f} at {float(getattr(contact, 'commanded_speed', current_speed) or 0.0):.1f} kt")


def request_ship_speed(contact, speed_kts):
    try:
        speed_kts = max(0.0, float(speed_kts))
    except (TypeError, ValueError):
        speed_kts = max(0.0, float(getattr(contact, "commanded_speed", getattr(contact, "speed", 0.0)) or 0.0))
    ensure_ship_command_state(contact)
    pause_ship_route_for_manual(contact, "speed command")
    contact.commanded_speed = speed_kts
    if speed_kts > 0.2:
        contact.ship_underway_speed = speed_kts
    print(f"Ship track {getattr(contact, 'track_number', '?')} requested speed {speed_kts:.1f} kt")


def set_ship_route_speed(contact, speed_kts):
    try:
        speed_kts = max(0.0, float(speed_kts))
    except (TypeError, ValueError):
        speed_kts = max(0.0, float(getattr(contact, "route_speed_kts", getattr(contact, "speed", 0.0)) or 0.0))
    if len(getattr(contact, "route_waypoints", []) or []) < 2:
        print(f"Ship track {getattr(contact, 'track_number', '?')} has no route speed to set")
        return False
    contact.route_speed_kts = speed_kts
    if getattr(contact, "route_active", False):
        contact.commanded_speed = speed_kts
    autosave_ship_route_to_config(contact)
    print(f"Ship track {getattr(contact, 'track_number', '?')} enroute speed set to {speed_kts:.1f} kt")
    return True


def resume_ship_route(contact):
    points = getattr(contact, "route_waypoints", []) or []
    if len(points) < 2:
        print(f"Ship track {getattr(contact, 'track_number', '?')} has no route to resume")
        return False
    speed_kts = max(0.0, float(getattr(contact, "route_speed_kts", getattr(contact, "speed", 0.0)) or 0.0))
    if speed_kts <= 0.0:
        speed_kts = max(ship_default_underway_speed_kts, float(getattr(contact, "ship_underway_speed", 0.0) or 0.0), float(getattr(contact, "speed", 0.0) or 0.0))
        contact.route_speed_kts = speed_kts
    if not getattr(contact, "route_loop", False) and int(getattr(contact, "route_index", 0) or 0) >= len(points) - 1:
        contact.route_index = max(0, len(points) - 2)
    contact.route_active = True
    contact.route_manual_paused = False
    contact.route_status = "Route resumed"
    update_ship_route_following(contact)
    autosave_ship_route_to_config(contact)
    print(f"Ship track {getattr(contact, 'track_number', '?')} resumed route at {speed_kts:.1f} kt")
    return True


def current_aircraft_heading_deg(default=0.0):
    try:
        return float(hdg or default) % 360.0
    except (TypeError, ValueError):
        return float(default) % 360.0


def contact_context_dropdown_rect(kind):
    if kind == "type":
        return pygame.Rect((530, 580), (180, 30))
    if kind == "class":
        return pygame.Rect((720, 580), (205, 30))
    if kind == "country":
        return pygame.Rect((720, 620), (205, 30))
    return pygame.Rect((530, 620), (180, 30))

def draw_contact_menu_ui(contact, y):

    contact.type_dropdown = pygame_gui.elements.UIDropDownMenu(
        ["Biological", "Submarine", "Surface-Ship", "Unknown"],
        "Unknown",
        pygame.Rect(570, y + 540, 150, 25),
        manager
    )

    contact.class_dropdown = pygame_gui.elements.UIDropDownMenu(
        contact_possible_type_list["Unknown"],
        "Unknown",
        pygame.Rect(740, y + 540, 150, 25),
        manager
    )

    register_ui_element(contact.type_dropdown, pygame.Rect(570, y + 540, 150, 25))
    register_ui_element(contact.class_dropdown, pygame.Rect(740, y + 540, 150, 25))
    

    contact.ui_created = True
    
contact_menu_font = scaled_sys_font(17)

def draw_contact_menu():
    menu_surface = pygame.Surface((860, 400))
    menu_surface.fill((100,100,20))

    y = 30

    for contact in contacts:
        #if contact.detected:

        # create UI only once
        if contact.ui_drawn == False:
            draw_contact_menu_ui(contact, y)
            contact.ui_drawn = True
        label_strip = contact_menu_font.render(
            f"{'TRACK':<8}{'LAT':<12}{'LONG':<12}{'SPD':<8}{'DEPTH':<8}",
            True,
            (255,255,255)
        )

        contact_strip = contact_menu_font.render(
            f"{contact.track_number:<8}{contact.contact_lat:<12.3f}{contact.contact_long:<12.3f}{contact.speed:<8}{contact.depth:<8}",
            True,
            (255,255,255)
        )
        menu_surface.blit(label_strip, (10,0))
        menu_surface.blit(contact_strip, (10, y))
        y += 30

    data_Surface.blit(menu_surface, (50,540))


def contact_tone_summary(contact):
    parts = []
    for tone in getattr(contact, "tones", []):
        if tone.label == "FMCW":
            continue
        parts.append(f"{tone.label}: {tone.freq:.0f}Hz")
    return ", ".join(parts) if parts else "None"


def contact_propulsor_summary(contact):
    blade_count = getattr(contact, "blade_count", None)
    shaft_rate = getattr(contact, "shaft_rate_hz", None)
    blade_rate = getattr(contact, "blade_rate_hz", None)
    if not blade_count or shaft_rate is None or blade_rate is None:
        return "Blades: N/A"
    return f"Blades: {blade_count} | Shaft: {shaft_rate:.1f} Hz | Blade: {blade_rate:.1f} Hz"

def reset_contact_classification(contact):
    contact.classification_type = "Unknown"
    contact.classification_class = "Unknown"
    contact.identity_status = "P"


def mark_contact_detected_unknown(contact):
    contact.detected = True
    if not getattr(contact, "operator_classified", False):
        reset_contact_classification(contact)


def update_contact_context_panel():
    global contact_context_last_text, contact_context_last_title

    if selected_contact is None:
        contact_context_panel.hide()
        torpedo_designate_button.hide()
        contact_lines_button.hide()
        hide_ship_command_controls()
        contact_delete_button.hide()
        contact_context_close_button.hide()
        contact_type_dropdown.hide()
        contact_class_dropdown.hide()
        contact_status_dropdown.hide()
        contact_country_dropdown.hide()
        contact_context_last_text = ""
        contact_context_last_title = ""
        return

    if contact_context_user_closed:
        contact_context_panel.hide()
        torpedo_designate_button.hide()
        contact_lines_button.hide()
        hide_ship_command_controls()
        contact_delete_button.hide()
        contact_context_close_button.hide()
        contact_type_dropdown.hide()
        contact_class_dropdown.hide()
        contact_status_dropdown.hide()
        contact_country_dropdown.hide()
        return

    if not contact_is_radar_visible(selected_contact):
        contact_context_panel.hide()
        torpedo_designate_button.hide()
        contact_lines_button.hide()
        hide_ship_command_controls()
        contact_delete_button.hide()
        contact_context_close_button.hide()
        contact_type_dropdown.hide()
        contact_class_dropdown.hide()
        contact_status_dropdown.hide()
        contact_country_dropdown.hide()
        contact_context_last_text = ""
        contact_context_last_title = ""
        return

    contact_context_panel.show()
    torpedo_designate_button.show()
    contact_lines_button.show()
    if selected_contact_is_surface_ship():
        show_ship_command_controls()
    else:
        hide_ship_command_controls()
    contact_delete_button.hide()
    contact_context_close_button.show()
    sync_torpedo_designate_button_style()
    sync_contact_lines_button_style()
    contact_type_dropdown.show()
    contact_class_dropdown.show()
    contact_status_dropdown.show()
    contact_country_dropdown.show()
    identity_status = getattr(selected_contact, "identity_status", "P")
    identity_label = contact_identity_labels.get(identity_status, "Pending")
    title_text = f"CONTACT {selected_contact.track_number}"
    info_text = (
        f"Track: {selected_contact.track_number}<br>"
        f"Type: {selected_contact.classification_type}<br>"
        f"Class: {selected_contact.classification_class}<br>"
        f"Identity: {identity_status} - {identity_label}<br>"
        f"Country: {getattr(selected_contact, 'country', 'Unknown')}<br>"
        f"Team: {getattr(selected_contact, 'team', 'Neutral')}<br>"
        f"Lat: {selected_contact.contact_lat:.5f}<br>"
        f"Lon: {selected_contact.contact_long:.5f}<br>"
        f"Speed: {selected_contact.speed:.1f} kt | Depth: {selected_contact.depth:.0f} m<br>"
        f"Command: {float(getattr(selected_contact, 'commanded_speed', selected_contact.speed) or 0):.1f} kt / {float(getattr(selected_contact, 'commanded_heading', selected_contact.bearing) or 0) % 360:03.0f} deg<br>"
        f"Route: {getattr(selected_contact, 'route_status', 'No route')} / {float(getattr(selected_contact, 'route_speed_kts', 0.0) or 0.0):.1f} kt<br>"
        f"Shadow: {getattr(selected_contact, 'shadow_status', getattr(selected_contact, 'shadow_target_name', '') or 'None')}<br>"
        f"Bearing: {selected_contact.bearing:.0f} deg | Buoys: {len(selected_contact.detecting_buoys)}<br>"
        f"Torpedo: {'DESIGNATED' if torpedo_designated_contact is selected_contact else 'None'}<br>"
        f"{contact_propulsor_summary(selected_contact)}<br>"
        f"Tones: {contact_tone_summary(selected_contact)}"
    )

    if title_text != contact_context_last_title:
        contact_context_title.set_text(title_text)
        contact_context_last_title = title_text
    if info_text != contact_context_last_text:
        contact_context_info.set_text(info_text)
        contact_context_last_text = info_text


def set_selected_contact(contact):
    global selected_contact, contact_type_dropdown, contact_class_dropdown, contact_status_dropdown, contact_country_dropdown, contact_context_user_closed, xbt_panel_visible

    selected_contact = contact
    contact_context_user_closed = False
    xbt_panel_visible = False
    xbt_panel_close_button.hide()
    if "xbt_raytrace_button" in globals():
        xbt_raytrace_button.hide()
    if "xbt_raytrace_clear_button" in globals():
        xbt_raytrace_clear_button.hide()
    if xbt_panel_select_dropdown is not None:
        xbt_panel_select_dropdown.hide()
    contact_context_panel.show()
    torpedo_designate_button.show()
    contact_lines_button.show()
    if selected_contact_is_surface_ship():
        show_ship_command_controls(update_heading_text=True)
    else:
        hide_ship_command_controls()
    contact_delete_button.hide()
    contact_context_close_button.show()
    sync_torpedo_designate_button_style()

    selected_type = getattr(contact, "classification_type", "Unknown")
    selected_class = getattr(contact, "classification_class", "Unknown")
    selected_status = getattr(contact, "identity_status", "P")
    selected_country = getattr(contact, "country", "Unknown")
    if selected_status not in contact_identity_options:
        selected_status = "P"
        contact.identity_status = selected_status
    if selected_country not in contact_country_options:
        selected_country = "Unknown"
        contact.country = selected_country
    class_options = contact_possible_type_list.get(selected_type, contact_possible_type_list["Unknown"])
    if selected_class not in class_options:
        selected_class = class_options[0]
        contact.classification_class = selected_class

    contact_type_dropdown.kill()
    contact_class_dropdown.kill()
    contact_status_dropdown.kill()
    contact_country_dropdown.kill()

    contact_type_dropdown = pygame_gui.elements.UIDropDownMenu(
        options_list=list(contact_possible_type_list.keys()),
        starting_option=selected_type,
        relative_rect=contact_context_dropdown_rect("type"),
        manager=manager,
        expansion_height_limit=180
    )
    contact_class_dropdown = pygame_gui.elements.UIDropDownMenu(
        options_list=class_options,
        starting_option=selected_class,
        relative_rect=contact_context_dropdown_rect("class"),
        manager=manager,
        expansion_height_limit=180
    )
    contact_status_dropdown = pygame_gui.elements.UIDropDownMenu(
        options_list=contact_identity_options,
        starting_option=selected_status,
        relative_rect=contact_context_dropdown_rect("status"),
        manager=manager,
        expansion_height_limit=180
    )
    contact_country_dropdown = pygame_gui.elements.UIDropDownMenu(
        options_list=contact_country_options,
        starting_option=selected_country,
        relative_rect=contact_context_dropdown_rect("country"),
        manager=manager,
        expansion_height_limit=180
    )
    update_contact_context_panel()


def internal_mouse_pos(screen_pos):
    x = (screen_pos[0] - display_viewport_rect.x) / max(0.0001, display_scale)
    y = (screen_pos[1] - display_viewport_rect.y) / max(0.0001, display_scale)
    return pygame.Vector2(x, y)


def find_contact_at_internal_pos(internal_pos):
    if internal_pos.x < INTERNAL_WIDTH / 2:
        return None

    map_pos = pygame.Vector2(internal_pos.x - INTERNAL_WIDTH / 2, internal_pos.y)
    click_radius = int(16 * map_overlay_symbol_scale())

    for contact in reversed(contacts):
        if not contact_is_radar_visible(contact):
            continue
        if any(tone.label == "FMCW" for tone in getattr(contact, "tones", [])):
            continue

        if display_mode == "RADAR":
            radar_width, radar_height = map_surf.get_size()
            center = pygame.Vector2(radar_width / 2, radar_height / 2)
            radar_radius = min(radar_width, radar_height) * 0.43
            radar_range_nm = radar_range_options[radar_range_index]
            pixels_per_nm = radar_radius / radar_range_nm
            rotation_deg = hdg if radar_orientation == "TRACK" else 0
            player_world = pygame.Vector2(latlong_to_pix(player_pos.x, player_pos.y))
            contact_world = pygame.Vector2(latlong_to_pix(contact.contact_lat, contact.contact_long))
            contact_pos = radar_point_from_world(contact_world, player_world, center, pixels_per_nm, rotation_deg)
            if center.distance_to(contact_pos) > radar_radius:
                continue
        else:
            contact_pos = pygame.Vector2(map_layer.translate_point(latlong_to_pix(contact.contact_lat, contact.contact_long)))

        if map_pos.distance_to(contact_pos) <= click_radius:
            return contact

    return None


def draw_sub_line(surf, color, center, radius, start_angle_deg, end_angle_deg, width=1):
    # Convert degrees → radians
    start_angle = math.radians(start_angle_deg)
    end_angle   = math.radians(end_angle_deg)

    # Calculate endpoints
    x1 = center[0] + radius * math.cos(start_angle)
    y1 = center[1] + radius * math.sin(start_angle)
    x2 = center[0] + radius * math.cos(end_angle)
    y2 = center[1] + radius * math.sin(end_angle)

   
    converted_x1,converted_y1 = map_layer.translate_point((x1,y1))
    converted_x2,converted_y2 = map_layer.translate_point((x2,y2))
    

    # Draw the 3 wedge edges
    """     pygame.draw.line(surf, color, center, (x1, y1), width)
        pygame.draw.line(surf, color, center, (x2, y2), width)
        pygame.draw.line(surf, color, (x1, y1),(x2, y2), width)  """

    return converted_x1,converted_y1,converted_x2,converted_y2 


def bearing_endpoint_screen(world_center, bearing_deg, offset_deg=0, length_nm=DIFAR_BEARING_DISPLAY_LENGTH_NM):
    world_center = pygame.Vector2(world_center)
    length_px = length_nm * pixels_per_nm_at(world_center)
    display_bearing = math.radians((bearing_deg + offset_deg) % 360)
    world_end = pygame.Vector2(
        world_center.x + math.sin(display_bearing) * length_px,
        world_center.y - math.cos(display_bearing) * length_px
    )
    return map_layer.translate_point(world_end)


def calc_bearing_uncert(highest_db, db_min=0, db_max=100, min_deg=1.0, max_deg=135.0, new_uncertainty = None):
    """
    Calculate bearing uncertainty in degrees based on highest received dB using a linear mapping.
    
    Parameters:
        highest_db : float
            Highest received signal level in dB.
        db_min : float
            Minimum expected dB (worst-case signal).
        db_max : float
            Maximum expected dB (best-case signal).
        min_deg : float
            Minimum possible uncertainty (best-case).
        max_deg : float
            Maximum possible uncertainty (worst-case).
    
    Returns:
        float: Bearing uncertainty in degrees.

    """
    # Clamp highest_db to expected range
    highest_db = max(db_min, min(highest_db, db_max))
    
    # Normalize between 0 and 1
    norm = (highest_db - db_min) / (db_max - db_min)
    
    # Linear mapping: higher dB → smaller uncertainty
    uncertainty = min_deg + (1.0 - norm) * (max_deg - min_deg)
    
    if uncertainty <= 30:
        new_uncertainty = 0
        

    return new_uncertainty if new_uncertainty is not None else uncertainty

colour_array = ["red"] * 512


class Active_Sonobuoy(pygame.sprite.Sprite):
    """Active DICASS-style buoy that emits a swept-frequency ping."""

    def __init__(self, position, depth, image, i, channel, start_freq, end_freq, sweep_time, source_db):
        super().__init__()
        self.image = pygame.Surface((1,1), pygame.SRCALPHA)
        self.image.fill((0,0,0,0))
        self.gap_time = 2
        self.icon_image = image.copy()
        self.rect = self.image.get_rect(center=position)
        self.x = position[0]
        self.y = position[1]
        self.line = None
        self.rect.x = position[0]
        self.rect.y = position[1]
        self.depth = depth
        self.corrected_range = None
        self.array_position = None
        self.db = None
        self.freq = None
        end_angle = math.radians(90)
        self.colour = colour_array[i]
        self.channel = channel       
        labele = font.render(str(self.channel),False,self.colour) 
        width, height = labele.get_size()
        new_label_size = width*1.1,height*1.3
        self.label = pygame.transform.scale(labele, new_label_size)


        self.start_khz = start_freq
        self.end_khz = end_freq
        self.sweep_time = sweep_time
        self.source_db = source_db
        self.bandwidth = end_freq - start_freq
        self.sweep_start_time = time.time()

    def update(self):
        self.icon_image.blit(self.label,(17,0))
        screen_pos = map_layer.translate_point(pygame.Vector2(self.x, self.y))

        self.icon_rect = self.icon_image.get_rect(center=screen_pos)
        map_overlay_surface.blit(self.icon_image, self.icon_rect)
    def generate_active_sonar_ping(self):
        """
        Generates one full active sonar cycle:
            - Sweep phase (ping)
            - Silent listen phase

        Returns:
            transmitting (bool)
            freq_hz (float or None)
            amplitude_db (float or None)
            progress (0-1 within sweep, or None if silent)
        """

        now = time.time()

        # Total cycle = transmit + silence
        total_cycle = self.sweep_time + self.gap_time

        elapsed = now - self.sweep_start_time

        # Reset cycle cleanly
        if elapsed >= total_cycle:
            self.sweep_start_time = now
            elapsed = 0

        # ----------------------------
        # SILENT PHASE
        # ----------------------------
        if elapsed > self.sweep_time:
            return False, 300, 0, 0

        # ----------------------------
        # TRANSMIT PHASE
        # ----------------------------
        progress = elapsed / self.sweep_time

        # Phase boundaries (45 / 45 / 10)
        phase1_end = 0.45
        phase2_end = 0.90

        base_start = self.start_khz
        base_end   = self.start_khz + self.bandwidth

        final_start = base_end * 1.1
        final_end   = final_start + self.bandwidth

        amplitude = self.source_db

        if progress <= phase1_end:
            local = progress / phase1_end
            freq_khz = base_start + (base_end - base_start) * local
            amplitude *= local  # fade in

        elif progress <= phase2_end:
            freq_khz = base_end
            amplitude = self.source_db

        else:
            local = (progress - phase2_end) / (1.0 - phase2_end)
            freq_khz = final_start + (final_end - final_start) * local
            amplitude *= (1.0 - local)  # fade out

        freq_hz = freq_khz * 1000

        return True, freq_hz, amplitude, progress

    def beat_frequency(self, target_range_nm, relative_velocity_m_s=0):
        """
        Estimate beat frequency for target at range and velocity (triangular FMCW).
        Uses only parameters from __init__: start_khz, end_khz, sweep_time.
        Returns beat frequency in kHz (realistic for small ranges).
        """
        c = 1500.0  # speed of sound in m/s
        R = target_range_nm * 1852.0  # nm -> meters

        # Convert kHz -> Hz for slope calculation
        start_hz = self.start_khz * 1e3
        end_hz = self.end_khz * 1e3
        slope_hz_per_s = (end_hz - start_hz) / self.sweep_time  # Hz/s

        # Time delay for round trip
        tau_range = 2 * R / c  # seconds
        f_range_hz = slope_hz_per_s * tau_range

        # Doppler contribution
        f_doppler_hz = slope_hz_per_s * (2 * relative_velocity_m_s / c) * self.sweep_time

        # Return in kHz
        return ((f_range_hz + f_doppler_hz) / 1e3) / 10
class Sonobuoy(pygame.sprite.Sprite):
    """Passive buoy: listens for contact tones and draws bearing uncertainty."""

    def __init__(self, position, depth, image, i, channel):
        super().__init__()
        self.image = pygame.Surface((1,1), pygame.SRCALPHA)
        self.image.fill((0,0,0,0))

        self.icon_image = image.copy()
        self.rect = self.image.get_rect(center=position)
        self.x = position[0]
        self.y = position[1]
        self.line = None
        self.rect.x = position[0]
        self.rect.y = position[1]
        self.depth = None
        self.corrected_range = None
        self.array_position = None
        self.db = None
        self.freq = None
        self.ray_array = []
        end_angle = math.radians(90)
        self.colour = colour_array[i]
        self.channel = channel       
        labele = font.render(str(self.channel),False,self.colour) 
        width, height = labele.get_size()
        new_label_size = width*1.1,height*1.3
        self.label = pygame.transform.scale(labele, new_label_size)
        self.hot = False
        self.range_circle = False
        self.bearing_lines_visible = True
        self.detections = []
        self.detected_contact = None
        self.bearing = 0
        self.next_detection_update = time.time() + random.uniform(0, DIFAR_DETECTION_UPDATE_INTERVAL_SEC)

    def update(self, offset, draw_map=True):
        now = time.time()
        should_refresh_detections = now >= getattr(self, "next_detection_update", 0.0)

        if should_refresh_detections:
            update_interval = DIFAR_DETECTION_UPDATE_INTERVAL_SEC if channel_is_selected_or_listened(self.channel) else DIFAR_HIDDEN_DETECTION_UPDATE_INTERVAL_SEC
            self.next_detection_update = now + update_interval + random.uniform(0, DIFAR_DETECTION_UPDATE_JITTER_SEC)
            candidate_detections = []
            sono_lat, sono_lon = pix_to_latlong(self.x, self.y)

            for contact in contacts:
                if is_dicass_ping_contact(contact) or not contact.tones:
                    continue

                contact_range_nm = haversine(
                    contact.contact_lat,
                    contact.contact_long,
                    sono_lat,
                    sono_lon
                )

                strongest_tone = None
                received_db = None
                best_snr = None
                for tone in contact.tones:
                    tone_received_db = difar_received_db(
                        tone.db,
                        contact_range_nm,
                        tone.freq,
                        getattr(contact, "depth", 0),
                        getattr(self, "depth", 0)
                    )
                    tone_snr = tone_received_db - difar_background_noise_db(tone.freq)
                    if tone_snr < DIFAR_MIN_SNR_DB:
                        continue
                    if best_snr is None or tone_snr > best_snr:
                        received_db = tone_received_db
                        best_snr = tone_snr
                        strongest_tone = tone

                if strongest_tone is None or received_db is None or best_snr is None:
                    continue

                bearing = haversine_bearing(
                    sono_lat,
                    sono_lon,
                    contact.contact_lat,
                    contact.contact_long
                )
                uncert = calc_difar_bearing_uncert_from_snr(best_snr, contact_range_nm)
                if uncert < 90:
                    candidate_detections.append({
                        "contact": contact,
                        "bearing": bearing,
                        "db": received_db,
                        "snr": best_snr,
                        "freq": doppler_shifted_frequency(strongest_tone.freq, contact, sono_lat, sono_lon),
                        "uncert": uncert
                    })

            candidate_detections.extend(iter_passive_active_difar_detections(sono_lat, sono_lon))

            candidate_detections.sort(key=lambda detection: detection["db"], reverse=True)
            self.detections = []
            for candidate in candidate_detections:
                masked = False
                for existing in self.detections:
                    bearing_gap = abs((candidate["bearing"] - existing["bearing"] + 180) % 360 - 180)
                    occlusion_width = max(
                        DIFAR_BEARING_RESOLUTION_DEG,
                        min(18.0, max(candidate["uncert"], existing["uncert"]) * 0.5)
                    )
                    if bearing_gap <= occlusion_width and candidate["snr"] <= existing["snr"] + 3.0:
                        masked = True
                        break
                if not masked:
                    self.detections.append(candidate)
                    if len(self.detections) >= DIFAR_MAX_DETECTIONS_PER_BUOY:
                        break

            strongest = self.detections[0] if self.detections else None
            self.db = strongest["db"] if strongest else None
            self.detected_contact = strongest["contact"] if strongest else None
            self.bearing = strongest["bearing"] if strongest else 0
            self.hot = any(detection["uncert"] == 0 for detection in self.detections)

        if not draw_map:
            return super().update()

        if bearing_lines_visible and getattr(self, "bearing_lines_visible", True):
            for detection in self.detections:
                if getattr(detection.get("contact"), "bearing_lines_hidden", False):
                    continue
                buoy_world = pygame.Vector2(self.x,self.y)
                buoy_screen = map_layer.translate_point(buoy_world)
                if detection["uncert"] <= 1.0:
                    center_end = bearing_endpoint_screen(buoy_world, detection["bearing"], offset)
                    detection["line"] = (center_end[0], center_end[1], center_end[0], center_end[1])
                    pygame.draw.line(map_overlay_surface, self.colour, buoy_screen, center_end, 2)
                else:
                    left_end = bearing_endpoint_screen(buoy_world, detection["bearing"] - detection["uncert"], offset)
                    right_end = bearing_endpoint_screen(buoy_world, detection["bearing"] + detection["uncert"], offset)
                    center_end = bearing_endpoint_screen(buoy_world, detection["bearing"], offset, DIFAR_BEARING_DISPLAY_LENGTH_NM * 0.85)
                    detection["line"] = (left_end[0], left_end[1], right_end[0], right_end[1])
                    fill_colour = pygame.Color(self.colour)
                    pygame.draw.polygon(
                        map_overlay_surface,
                        (fill_colour.r, fill_colour.g, fill_colour.b, 14),
                        [buoy_screen, left_end, right_end]
                    )
                    pygame.draw.line(map_overlay_surface, self.colour, buoy_screen, left_end, 2)
                    pygame.draw.line(map_overlay_surface, self.colour, buoy_screen, right_end, 2)
                    pygame.draw.line(map_overlay_surface, self.colour, left_end, right_end, 1)
                    pygame.draw.line(map_overlay_surface, self.colour, buoy_screen, center_end, 1)
        self.icon_image.blit(self.label,(17,0))

        screen_pos = map_layer.translate_point(pygame.Vector2(self.x, self.y))
        self.icon_rect = self.icon_image.get_rect(center=screen_pos)
        if sono.range_circle == True:
            circle_radius_px = DIFAR_REFERENCE_RANGE_NM * pixels_per_nm_at(pygame.Vector2(self.x, self.y)) * map_layer.zoom
            pygame.draw.circle(
            map_overlay_surface,   # surface to draw on
            (0, 255, 0),           # colour
            (int(screen_pos[0]), int(screen_pos[1])),  # centre
            max(1, int(circle_radius_px)),                    # radius in pixels
            1                      # line thickness (0 = filled)
            )
        
        map_overlay_surface.blit(self.icon_image, self.icon_rect)
        self.icon_image
        return super().update()
        
torp_array = []


def world_distance_nm(pos_a, pos_b):
    """Distance between two world-map pixel positions, measured in nautical miles."""
    lat_a, lon_a = pix_to_latlong(pos_a.x, pos_a.y)
    lat_b, lon_b = pix_to_latlong(pos_b.x, pos_b.y)
    return haversine(lat_a, lon_a, lat_b, lon_b)


def pixels_per_nm_at(pos):
    """Local map scale used only to convert an NM step into world pixels."""
    one_pixel_east = pygame.Vector2(pos.x + 1, pos.y)
    nm_per_pixel = world_distance_nm(pos, one_pixel_east)
    if nm_per_pixel <= 0:
        return 1
    return 1 / nm_per_pixel


def draw_dotted_line(surface, colour, start, end, width=1, dash_length=8, gap_length=6):
    start = pygame.Vector2(start)
    end = pygame.Vector2(end)
    delta = end - start
    distance = delta.length()
    if distance <= 0:
        return

    direction = delta / distance
    travelled = 0
    while travelled < distance:
        dash_start = start + direction * travelled
        dash_end = start + direction * min(travelled + dash_length, distance)
        pygame.draw.line(surface, colour, dash_start, dash_end, width)
        travelled += dash_length + gap_length


def draw_dotted_circle(surface, colour, center, radius, width=1, dash_degrees=7, gap_degrees=5):
    if radius <= 0:
        return

    center = pygame.Vector2(center)
    angle = 0
    while angle < 360:
        start_angle = math.radians(angle)
        end_angle = math.radians(min(angle + dash_degrees, 360))
        start = (
            center.x + math.sin(start_angle) * radius,
            center.y - math.cos(start_angle) * radius
        )
        end = (
            center.x + math.sin(end_angle) * radius,
            center.y - math.cos(end_angle) * radius
        )
        pygame.draw.line(surface, colour, start, end, width)
        angle += dash_degrees + gap_degrees


class Torpedo:
    """Stingray torpedo that homes on the nearest contact with a chosen tone."""

    def __init__(self, position, target_frequency, launch_heading, image, seeker_mode="PASSIVE"):
        self.pos = pygame.Vector2(position)
        self.target_frequency = float(target_frequency)
        self.seeker_mode = seeker_mode
        self.heading = launch_heading
        self.display_heading = launch_heading
        self.image = pygame.transform.smoothscale(image.copy(), (32, 32))
        self.speed_kts = 45
        self.hit_radius_nm = 0.08
        self.active = True
        self.finished = False
        self.detonated = False
        self.detonation_started = 0
        self.impact_display_time = 4.0
        self.max_run_time = 45
        self.max_turn_rate_deg_s = 28.0 if seeker_mode == "ACTIVE" else 18.0
        self.launch_time = time.time()
        self.target = None
        self.destroyed_target = None
        self.trail = []
        self.last_target_track = None
        self.last_passive_track_print = {}

    def passive_contact_trackable(self, contact):
        if not getattr(contact, "tones", None):
            return False
        contact_pos = pygame.Vector2(latlong_to_pix(contact.contact_lat, contact.contact_long))
        distance_nm = world_distance_nm(self.pos, contact_pos)
        best_snr = None
        best_frequency_error = None
        for tone in contact.tones:
            frequency_error = abs(tone.freq - self.target_frequency)
            if frequency_error > max(180.0, self.target_frequency * 0.22):
                continue
            received_db = difar_received_db(
                tone.db,
                distance_nm,
                tone.freq,
                getattr(contact, "depth", 0),
                0
            )
            snr = received_db - difar_background_noise_db(tone.freq)
            if best_snr is None or snr > best_snr:
                best_snr = snr
                best_frequency_error = frequency_error
        if best_snr is None:
            return False
        if best_snr < 8.0:
            track_number = getattr(contact, "track_number", "----")
            now = time.time()
            if now - self.last_passive_track_print.get(track_number, 0.0) > 3.0:
                self.last_passive_track_print[track_number] = now
                print(
                    f"[TORP PASSIVE] lost/ignored track {track_number} "
                    f"SNR {best_snr:.1f} dB at {distance_nm:.2f} NM"
                )
            return False
        return True

    def find_target(self, contact_list):
        best_contact = None
        best_score = None
        designated = globals().get("torpedo_designated_contact")

        if designated in contact_list and not is_dicass_ping_contact(designated):
            if self.seeker_mode == "ACTIVE" or self.passive_contact_trackable(designated):
                self.target = designated
                self.last_target_track = getattr(designated, "track_number", None)
                return

        for contact in contact_list:
            if is_dicass_ping_contact(contact):
                continue

            contact_pos = pygame.Vector2(latlong_to_pix(contact.contact_lat, contact.contact_long))
            distance = world_distance_nm(self.pos, contact_pos)
            if self.seeker_mode == "ACTIVE":
                score = distance
            else:
                if not self.passive_contact_trackable(contact):
                    continue
                frequency_error = min(abs(tone.freq - self.target_frequency) for tone in contact.tones)
                score = distance + (frequency_error * 0.001)

            if best_score is None or score < best_score:
                best_score = score
                best_contact = contact

        if self.seeker_mode == "PASSIVE" and self.target is not None and best_contact is None:
            print(f"[TORP PASSIVE] lost target {getattr(self.target, 'track_number', '----')} after quieting/evasion")
        self.target = best_contact
        self.last_target_track = getattr(best_contact, "track_number", None) if best_contact is not None else None

    def update(self, dt_seconds, contact_list, draw_map=True):
        global selected_contact, torpedo_designated_contact

        if self.finished:
            return

        if self.detonated:
            if draw_map:
                self.draw_impact()
            if time.time() - self.detonation_started > self.impact_display_time:
                self.finished = True
            return

        if time.time() - self.launch_time > self.max_run_time:
            self.finished = True
            return

        step_nm = (self.speed_kts / 3600) * dt_seconds
        self.find_target(contact_list)
        if self.target is None:
            # No acoustic target yet: run straight from the launch heading so
            # the weapon still visibly leaves the aircraft.
            step_pixels = step_nm * pixels_per_nm_at(self.pos)
            self.pos.x += math.sin(math.radians(self.heading)) * step_pixels
            self.pos.y += -math.cos(math.radians(self.heading)) * step_pixels
            append_fading_trail(self, self.pos, max_points=40)
            if draw_map:
                self.draw()
            return

        target_pos = pygame.Vector2(latlong_to_pix(self.target.contact_lat, self.target.contact_long))
        distance_nm = world_distance_nm(self.pos, target_pos)
        desired_heading = haversine_bearing(
            pix_to_latlong(self.pos.x, self.pos.y)[0],
            pix_to_latlong(self.pos.x, self.pos.y)[1],
            self.target.contact_lat,
            self.target.contact_long
        )
        heading_error = (desired_heading - self.heading + 180) % 360 - 180
        max_turn = self.max_turn_rate_deg_s * dt_seconds
        self.heading = (self.heading + max(-max_turn, min(max_turn, heading_error))) % 360

        if distance_nm <= self.hit_radius_nm:
            self.destroyed_target = self.target
            self.target.detected = False
            self.target.broadcasting = False
            if selected_contact is self.target:
                selected_contact = None
            if torpedo_designated_contact is self.target:
                torpedo_designated_contact = None
            if self.target in contacts:
                destroyed_track = getattr(self.target, "track_number", None)
                if destroyed_track is not None:
                    remove_injected_ship(destroyed_track)
                contacts.remove(self.target)
            self.target = None
            self.detonated = True
            self.detonation_started = time.time()
            return

        step_pixels = step_nm * pixels_per_nm_at(self.pos)
        self.pos.x += math.sin(math.radians(self.heading)) * step_pixels
        self.pos.y += -math.cos(math.radians(self.heading)) * step_pixels

        append_fading_trail(self, self.pos, max_points=40)
        if draw_map:
            self.draw()

    def draw(self):
        draw_fading_trail(map_overlay_surface, self.trail, map_layer.translate_point, (120, 210, 255), 2)

        screen_pos = map_layer.translate_point(self.pos)
        pygame.draw.circle(map_overlay_surface, (120, 210, 255), screen_pos, 10, 1)

        # Bearing pointer from the torpedo nose. This is drawn in overlay/screen
        # space, but the bearing itself comes from geographic target geometry.
        display_error = (self.heading - self.display_heading + 180) % 360 - 180
        self.display_heading = (self.display_heading + max(-6.0, min(6.0, display_error))) % 360
        pointer_end = (
            screen_pos[0] + math.sin(math.radians(self.display_heading)) * 34,
            screen_pos[1] - math.cos(math.radians(self.display_heading)) * 34
        )
        pygame.draw.line(map_overlay_surface, (255, 230, 120), screen_pos, pointer_end, 2)

        draw_target = self.target if self.target is not None else self.destroyed_target
        if draw_target is not None:
            target_pos = pygame.Vector2(latlong_to_pix(draw_target.contact_lat, draw_target.contact_long))
            target_screen_pos = map_layer.translate_point(target_pos)
            draw_dotted_line(map_overlay_surface, (255, 230, 120), screen_pos, target_screen_pos, 1)
            pygame.draw.circle(map_overlay_surface, (255, 230, 120), target_screen_pos, 18, 2)
            pygame.draw.line(
                map_overlay_surface,
                (255, 230, 120),
                (target_screen_pos[0] - 24, target_screen_pos[1]),
                (target_screen_pos[0] + 24, target_screen_pos[1]),
                1
            )
            pygame.draw.line(
                map_overlay_surface,
                (255, 230, 120),
                (target_screen_pos[0], target_screen_pos[1] - 24),
                (target_screen_pos[0], target_screen_pos[1] + 24),
                1
            )
            lock_label = font.render("LOCK", False, (255, 230, 120))
            map_overlay_surface.blit(lock_label, (target_screen_pos[0] + 12, target_screen_pos[1] - 28))

        rect = self.image.get_rect(center=screen_pos)
        map_overlay_surface.blit(self.image, rect)

    def draw_impact(self):
        screen_pos = map_layer.translate_point(self.pos)
        elapsed = time.time() - self.detonation_started
        draw_fading_trail(map_overlay_surface, self.trail, map_layer.translate_point, (120, 210, 255), 2)
        radius = int(12 + elapsed * 24)
        pygame.draw.circle(map_overlay_surface, (255, 230, 120), screen_pos, radius, 2)
        pygame.draw.circle(map_overlay_surface, (255, 110, 80), screen_pos, max(3, radius // 3), 1)
        pygame.draw.line(map_overlay_surface, (255, 230, 120), (screen_pos[0] - 12, screen_pos[1]), (screen_pos[0] + 12, screen_pos[1]), 2)
        pygame.draw.line(map_overlay_surface, (255, 230, 120), (screen_pos[0], screen_pos[1] - 12), (screen_pos[0], screen_pos[1] + 12), 2)

def launch_torp():
    global sonoArmed
    if sonoArmed == True:
        play_launch_sound()
        launch_pos = latlong_to_pix(player_pos.x, player_pos.y)
        torp_array.append(Torpedo(launch_pos, selected_torpedo_frequency, hdg, torpedo_surface, selected_torpedo_mode))
        sonoArmed = False
        sync_arm_button_style()




sono_duration = 5000  
        
sono_channel_array = ["None"]
active_sonobuoy = False
pending_sono_launch_channel = None
xbt_exists = False
timer = Timer(0.1 * 60)

def launch_xbt():
    global sonoArmed, xbt_counter, latest_xbt_profile, xbt_panel_selected_label
    global xbt_exists
    if sonoArmed == True:
        play_launch_sound()
        xbt_channel = displayed_channel
        bt_sono = pygame.Vector2(latlong_to_pix(player_pos.x, player_pos.y))
        spawn_splash(bt_sono)
        spawn_msfs_splash(player_pos.x, player_pos.y)
        xbt_counter += 1
        xbt_label = f"CH {xbt_channel}"
        local_thermocline = max(180, min(seabed_depth - 250, thermocline_depth + random.uniform(-120, 120)))
        local_surface_temp = surface_temp + random.uniform(-1.2, 1.2)
        local_seabed_temp = seabed_temp + random.uniform(-0.5, 0.5)
        xbt_profile = XBT(
            temp_surface=local_surface_temp,
            temp_seabed=local_seabed_temp,
            thermocline_depth=local_thermocline,
            max_depth=seabed_depth,
            position=bt_sono,
            label=xbt_label
        )
        xbt_profile.update()
        latest_xbt_profile = xbt_profile
        xbt_profiles[xbt_label] = xbt_profile
        xbt_array.append(bt_sono)
        sonoArmed = False
        sync_arm_button_style()
        xbt_exists = True
        xbt_panel_selected_label = xbt_label
        update_xbt_panel_selector()
        advance_displayed_channel_after_launch(xbt_channel)
        timer.start()
            
            
map_path = "Tiled.tmx"
map_surfaces = list()
tmx_data = load_pygame(map_path)

map_surfaces.append((map_surf, map_surf.get_size(), 0))

# create new renderer (camera)

map_surface = pygame.Surface((1920/2,1080),pygame.HWSURFACE | pygame.DOUBLEBUF)                              
map_layer = pyscroll.BufferedRenderer(
    data=pyscroll.data.TiledMapData(tmx_data),
    size=map_surface.get_size(),
    clamp_camera=False,
)
map_width = tmx_data.width * tmx_data.tilewidth
map_height = tmx_data.height * tmx_data.tileheight

full_map = pygame.Surface((map_width, map_height)).convert_alpha()
full_map.fill((0, 0, 0, 0))  # optional transparent background

map_rect = full_map.get_rect()
map_layer.draw(full_map, map_rect)


camera_offset = pygame.Vector2(0, 0)
is_panning = False
last_mouse_pos = pygame.Vector2(0, 0)

map_layer.zoom = 1

map_group = PyscrollGroup(map_layer=map_layer, default_layer=1)

def launch_active_sonobuoy():
    global sonoArmed, sono_time_up, active_sonobuoy
    play_launch_sound()
    if sonoArmed == True:

        sono_timer.start()
        active_sonobuoy = True
        
def launch_sonobuoy():
    global sonoArmed, sono_time_up
    play_launch_sound()

    
    if sonoArmed == True:
        active_sonobuoy = False
        sono_timer.start()



def arm_sonobuoy():
    global sonoArmed
    sonoArmed = not sonoArmed
    sync_arm_button_style()

    
    


def freq_to_x(freq, max_freq=3500, gram_width=450):
    return int((freq / max_freq) * gram_width)


def bearing_hue_colour(bearing_deg, intensity=1.0):
    intensity = max(0.0, min(1.0, float(intensity)))
    colour = pygame.Color(0, 0, 0)
    colour.hsva = (float(bearing_deg) % 360, 100, max(8, int(100 * intensity)), 100)
    return colour


def draw_azigram_colour_key(surface, center=(448, 15), radius=12):
    center_vec = pygame.Vector2(center)
    for bearing in range(0, 360, 4):
        angle = math.radians(bearing)
        end = (
            center_vec.x + math.sin(angle) * radius,
            center_vec.y - math.cos(angle) * radius
        )
        pygame.draw.line(surface, bearing_hue_colour(bearing, 1.0), center_vec, end, 1)
    pygame.draw.circle(surface, (210, 210, 210), center, radius, 1)
    key_font = getattr(draw_azigram_colour_key, "font", None)
    if key_font is None:
        key_font = scaled_sys_font(9, bold=True)
        draw_azigram_colour_key.font = key_font
    for text, pos in (
        ("N", (center[0] - 3, center[1] - radius - 9)),
        ("E", (center[0] + radius + 2, center[1] - 5)),
        ("S", (center[0] - 3, center[1] + radius - 1)),
        ("W", (center[0] - radius - 8, center[1] - 5)),
    ):
        surface.blit(key_font.render(text, False, (220, 220, 220)), pos)


def build_azigram_row(source_row, detections, width=450):
    row = pygame.Surface((width, 1))
    detection_bins = []
    for detection in detections:
        x = int(max(0, min(width - 1, (detection["freq"] / (fs / 2)) * (width - 1))))
        snr = max(DIFAR_MIN_SNR_DB, min(DIFAR_CLEAR_SNR_DB, detection.get("snr", DIFAR_MIN_SNR_DB)))
        snr_intensity = (snr - DIFAR_MIN_SNR_DB) / max(1, DIFAR_CLEAR_SNR_DB - DIFAR_MIN_SNR_DB)
        detection_bins.append((x, detection["bearing"], snr_intensity))

    for x in range(width):
        pixel = source_row.get_at((x, 0))
        amplitude = max(pixel.r, pixel.g, pixel.b) / 255.0

        sin_sum = 0.0
        cos_sum = 0.0
        weight_sum = 0.0
        intensity_boost = 0.0
        for detection_x, bearing, snr_intensity in detection_bins:
            distance = abs(x - detection_x)
            if distance > 18:
                continue
            falloff = math.exp(-((distance / 7.0) ** 2))
            weight = falloff * max(0.05, snr_intensity)
            bearing_rad = math.radians(bearing)
            sin_sum += math.sin(bearing_rad) * weight
            cos_sum += math.cos(bearing_rad) * weight
            weight_sum += weight
            intensity_boost = max(intensity_boost, falloff * snr_intensity)

        if weight_sum > 0:
            bearing = math.degrees(math.atan2(sin_sum, cos_sum)) % 360
            intensity = max(amplitude, 0.18 + 0.82 * intensity_boost)
            colour = bearing_hue_colour(bearing, intensity)
        else:
            if amplitude < 0.025 and random.random() > 0.08:
                colour = pygame.Color(0, 0, 0)
            else:
                noise_bearing = random.uniform(0, 360)
                intensity = max(0.04, min(0.45, amplitude * random.uniform(0.35, 0.9)))
                colour = bearing_hue_colour(noise_bearing, intensity)

        row.set_at((x, 0), colour)

    return row


def channel_has_azigram_slot(channel):
    return any(
        slot.active and
        slot.display_mode == "AZIGRAM" and
        slot.selected not in (None, "None") and
        str(slot.selected).isdigit() and
        int(slot.selected) == int(channel)
        for slot in spectrogram_slot_array
    )


def narrowband_window_for_channel(channel):
    if not is_numeric_channel(channel):
        return None
    channel_int = int(channel)
    windows = []
    for slot in spectrogram_slot_array:
        if (
            slot.active and
            slot.band_mode == "NARROWBAND" and
            is_numeric_channel(slot.selected) and
            int(slot.selected) == channel_int
        ):
            window = slot.current_frequency_window()
            if window is not None:
                windows.append(window)
    if not windows:
        return None
    return min(windows, key=lambda item: item[1] - item[0])


def narrowband_nfft_for_window(window, signal_len):
    if window is None:
        return None
    width_hz = max(1.0, float(window[1] - window[0]))
    if width_hz <= 100:
        target = 32768
    elif width_hz <= 250:
        target = 24576
    elif width_hz <= 500:
        target = 16384
    elif width_hz <= 1000:
        target = 8192
    else:
        target = 4096
    return max(int(signal_len), target)


def is_numeric_channel(value):
    return value is not None and str(value).isdigit()


def channel_is_selected_or_listened(channel):
    if not is_numeric_channel(channel):
        return False
    channel_int = int(channel)
    for slot in spectrogram_slot_array:
        if is_numeric_channel(slot.selected) and int(slot.selected) == channel_int:
            return True
    with listen_audio_lock:
        listening_slot = listening_spectrogram_slot
    return listening_slot is not None and is_numeric_channel(listening_slot.selected) and int(listening_slot.selected) == channel_int


def get_xbt_profile_for_selection(selection):
    if selection in (None, "None"):
        return None
    return xbt_profiles.get(str(selection))


sono_control_surface.fill((5,5,10))


signal = 0


class SpectrogramUI:
    """Scrolling spectrogram image for one deployed sonobuoy."""

    def __init__(self, sono):
        self.sono = sono
        self.ui_surface = pygame.Surface((480,270),masks=(255,255,0))
        self.gram_surface = pygame.Surface((450,200))
        self.azi_surface = pygame.Surface((450,200))
        self.gram_highres_surface = pygame.Surface((1024, 200))
        self.azi_highres_surface = pygame.Surface((1024, 200))
        self.azi_surface.fill((0, 0, 0))
        self.gram_highres_surface.fill((0, 0, 0))
        self.azi_highres_surface.fill((0, 0, 0))
        self.current_time_sec = time.time()
        self.time_window_sec = 1.0  # how much time the spectrogram shows
        self.current_time_sec = time.time()
        self.spec_x, self.spec_y = 15, 30
        self.spec_width, self.spec_height = 450, 200
        self.tick_font = scaled_sys_font(12)
        self.tooltip_font = scaled_sys_font(14)
        self.border_colour = (92, 96, 98)

    def render_surface(self, display_mode, freq_window=None):
        self.ui_surface.fill((5,5,10))
        pygame.draw.rect(self.ui_surface, self.border_colour, self.ui_surface.get_rect(), 1)
        if freq_window is not None:
            active_plot_surface = self.azi_highres_surface if display_mode == "AZIGRAM" else self.gram_highres_surface
        else:
            active_plot_surface = self.azi_surface if display_mode == "AZIGRAM" else self.gram_surface
        spec_x, spec_y = 15, 64
        spec_width, spec_height = 450, 156
        max_freq = fs / 2
        if freq_window is None:
            low_freq, high_freq = 0.0, max_freq
            src_h = active_plot_surface.get_height()
            src_rect = pygame.Rect(0, max(0, src_h - spec_height), active_plot_surface.get_width(), min(spec_height, src_h))
            plot_image = pygame.transform.smoothscale(active_plot_surface.subsurface(src_rect), (spec_width, spec_height))
            self.ui_surface.blit(plot_image, (spec_x, spec_y))
        else:
            low_freq, high_freq = freq_window
            low_freq = max(0.0, min(float(low_freq), max_freq - 1.0))
            high_freq = max(low_freq + 25.0, min(float(high_freq), max_freq))
            src_w = active_plot_surface.get_width()
            src_h = active_plot_surface.get_height()
            src_left = int((low_freq / max_freq) * src_w)
            src_right = int((high_freq / max_freq) * src_w)
            src_right = max(src_left + 1, min(src_w, src_right))
            src_rect = pygame.Rect(src_left, 0, src_right - src_left, src_h)
            zoomed = pygame.transform.smoothscale(active_plot_surface.subsurface(src_rect), (spec_width, spec_height))
            self.ui_surface.blit(zoomed, (spec_x, spec_y))

        num_freq_ticks = 5 if freq_window is not None else 7
        for i in range(num_freq_ticks + 1):
            x = spec_x + int(i * (spec_width / num_freq_ticks))
            tick_value = int(low_freq + (i / num_freq_ticks) * (high_freq - low_freq))
            pygame.draw.line(self.ui_surface, (200, 200, 200), (x, spec_y + spec_height), (x, spec_y + spec_height + 5), 1)
            label = self.tick_font.render(f"{tick_value}", True, (200, 200, 200))
            self.ui_surface.blit(label, (x - label.get_width() // 2, spec_y + spec_height + 8))

        return self.ui_surface

    def update(self):





        self.latest_row_signal = generate_signal_chunk(
            fs,
            SPECTROGRAM_CHUNK_SECONDS,
            contacts,
            random.randint(60, 85),
            (1, min(3499, fs / 2 - 1)),
            self.sono.channel
        )


        self.latest_row= generate_latest_spectrogram_row(
            self.latest_row_signal,
            fs=fs,
            nperseg=2048,
            noverlap=1536
            )
        narrow_window = narrowband_window_for_channel(self.sono.channel)
        highres_nfft = narrowband_nfft_for_window(narrow_window, len(self.latest_row_signal))
        if highres_nfft is not None:
            self.latest_highres_row = generate_latest_spectrogram_row(
                self.latest_row_signal,
                fs=fs,
                nperseg=min(len(self.latest_row_signal), 4096),
                noverlap=max(0, min(len(self.latest_row_signal), 4096) - 512),
                nfft=highres_nfft
                )
        else:
            self.latest_highres_row = None

            # Scroll left and blit new column
    # Scroll spectrogram surface UP by 1 pixel
        self.gram_surface.scroll(dx=0, dy=-1)

        # Clear the bottom row to avoid artifacts

        #scrolling_spectrogram_surface.fill((255, 0, 0))  # TEMP: force red for debug
        last_save_time = self.current_time_sec
        # Blit the latest row at the bottom (x=0, y=height-1)
        if self.latest_highres_row is not None:
            highres_width = max(1, self.latest_highres_row.get_width())
            if self.gram_highres_surface.get_width() != highres_width:
                self.gram_highres_surface = pygame.Surface((highres_width, self.gram_surface.get_height()))
                self.azi_highres_surface = pygame.Surface((highres_width, self.azi_surface.get_height()))
                self.gram_highres_surface.fill((0, 0, 0))
                self.azi_highres_surface.fill((0, 0, 0))
            self.gram_highres_surface.scroll(dx=0, dy=-1)
            self.gram_highres_surface.blit(self.latest_highres_row, (0, self.gram_highres_surface.get_height() - 1))
        self.scaled_latest_row = pygame.transform.scale(self.latest_row,(450,1))
        if channel_has_azigram_slot(self.sono.channel):
            self.azi_surface.scroll(dx=0, dy=-1)
            pygame.draw.rect(self.azi_surface, (0, 0, 0), pygame.Rect(0, self.azi_surface.get_height() - 1, self.azi_surface.get_width(), 1))
            azigram_row = build_azigram_row(self.scaled_latest_row, getattr(self.sono, "detections", []), 450)
            self.azi_surface.blit(azigram_row, (0, self.azi_surface.get_height() - 1))
            if self.latest_highres_row is not None:
                self.azi_highres_surface.scroll(dx=0, dy=-1)
                pygame.draw.rect(self.azi_highres_surface, (0, 0, 0), pygame.Rect(0, self.azi_highres_surface.get_height() - 1, self.azi_highres_surface.get_width(), 1))
                azigram_highres_row = build_azigram_row(self.latest_highres_row, getattr(self.sono, "detections", []), self.azi_highres_surface.get_width())
                self.azi_highres_surface.blit(azigram_highres_row, (0, self.azi_highres_surface.get_height() - 1))
        if self.sono.hot:
            self.border_colour = (153, 201, 121)

            #self.border_colour = (177, 59, 59) RED COLOUR
        else:
            self.border_colour = (92, 96, 98)
        self.gram_surface.blit(self.scaled_latest_row, (0, self.gram_surface.get_height() - 1))
        if torpedo_explosion_audible(self.sono.channel):
            pygame.draw.rect(
                self.gram_surface,
                (255, 210, 80),
                pygame.Rect(0, self.gram_surface.get_height() - 8, self.gram_surface.get_width(), 8)
            )
        if water_impact_audible(self.sono.channel):
            pygame.draw.rect(
                self.gram_surface,
                (170, 225, 255),
                pygame.Rect(0, self.gram_surface.get_height() - 5, self.gram_surface.get_width(), 5)
            )
        self.render_surface("SPECTROGRAM")

        #data_Surface.blit(self.ui_surface,(0,0))


def add_manual_azigram_bearing_line(sono, detection):
    if sono is None or detection is None:
        return
    manual_azigram_bearing_lines.append({
        "sono": sono,
        "bearing": float(detection.get("bearing", 0.0)),
        "freq": float(detection.get("freq", 0.0)),
        "label": str(detection.get("label", "AZI")),
        "created": time.time(),
    })
    del manual_azigram_bearing_lines[:-12]


def draw_manual_azigram_bearing_lines_map(surface):
    for item in manual_azigram_bearing_lines:
        sono = item.get("sono")
        if sono not in sono_array:
            continue
        start_world = pygame.Vector2(sono.x, sono.y)
        start_lat, start_lon = pix_to_latlong(start_world.x, start_world.y)
        end_lat, end_lon = destination_from_bearing(start_lat, start_lon, item.get("bearing", 0.0), DIFAR_BEARING_DISPLAY_LENGTH_NM)
        end_world = pygame.Vector2(latlong_to_pix(end_lat, end_lon))
        start_screen = map_layer.translate_point(start_world)
        end_screen = map_layer.translate_point(end_world)
        draw_dotted_line(surface, (255, 230, 90), start_screen, end_screen, 2, dash_length=8, gap_length=5)
        label = font.render(f"{item.get('bearing', 0):03.0f} {item.get('freq', 0):.0f}", False, (255, 230, 90))
        surface.blit(label, (end_screen[0] + 4, end_screen[1] - 8))


def draw_manual_azigram_bearing_lines_radar(surface, center, radar_radius, rotation_deg):
    player_world = pygame.Vector2(latlong_to_pix(player_pos.x, player_pos.y))
    pixels_per_nm = radar_radius / radar_range_options[radar_range_index]
    for item in manual_azigram_bearing_lines:
        sono = item.get("sono")
        if sono not in sono_array:
            continue
        sono_screen = radar_point_from_world(pygame.Vector2(sono.x, sono.y), player_world, center, pixels_per_nm, rotation_deg)
        if center.distance_to(sono_screen) <= radar_radius:
            draw_radar_bearing_detection(surface, sono_screen, item.get("bearing", 0.0), 0.0, rotation_deg, radar_radius, colour=(255, 230, 90))


#1920,1080            960, 480

class SpectrogramSlot:
    """One visible display slot that can be assigned to a sonobuoy channel."""

    def __init__(self, topleft, index):
        self.topleft = topleft
        self.index = index
        self.selected = None        # which channel is assigned
        self.display_mode = "SPECTROGRAM"
        self.band_mode = "BROADBAND"
        self.narrowband_window = None
        self.narrowband_drag_start_freq = None
        self.narrowband_drag_current_freq = None
        self.narrowband_drag_base_window = None
        self.marker_type = "Marks Off"
        self.marker_class = "None"
        self.active = True          # <-- NEW flag: whether the slot is active
        self._last_in_menu = None
        self.bearing_lines_visible = True
        self.update_ui()
        self.disable_slot_ui()
        self.last_audio_time = 0  # timestamp of last audi
        self.ui_slot_surface = pygame.Surface((480,270))
        self.crosshair_surface = pygame.Surface((480,270))
        self.marker_font = scaled_sys_font(10, bold=True)

        data_Surface.blit(self.ui_slot_surface, topleft)
        self.time_window_sec = 6
        self.class_rects = {
        "Kilo": [freq_to_x(250),freq_to_x(1000),freq_to_x(1500)],
        "Akula": [freq_to_x(350),freq_to_x(1500),freq_to_x(2500)],
        "Delta": [freq_to_x(500),freq_to_x(2000),freq_to_x(2800)],
        "Borei": [freq_to_x(300),freq_to_x(1500),freq_to_x(2500)],
        "Yasen": [freq_to_x(500),freq_to_x(1700),freq_to_x(3000)]
        }
    def disable_slot_ui(self):
        # Showing/hiding pygame_gui elements is relatively expensive. Only apply
        # visibility changes when the app actually switches menu/game state.
        if self._last_in_menu == in_menu:
            return
        self._last_in_menu = in_menu

        if not in_menu:
            self.channel_dropdown.show()
            self.scuttle_button.show()

            depth_dropdown.show()
            launch_button.show()
            arm_button.show()
            auto_buoy_button.show()
            auto_buoy_status_label.show()

            sonobuoy_dropdown.show()
            sync_torpedo_control_visibility()
            self.difar_display_button.show()
            self.band_mode_button.show()
            self.listen_button.show()
            self.toggle_range_circle_button.show()
            self.toggle_bearing_lines_button.show()
            self.marker_type_dropdown.show()
            self.marker_class_dropdown.show()
            controls_panel.show()
            #IN MENU BUTTONS

            simulator_button.hide()

            save_button.hide()
            load_button.hide()
            civilian_traffic_button.hide()
            whale_traffic_button.hide()

            start_button.hide()
            map_mode_button.show()
            radar_mode_button.show()
            radar_orientation_button.show()
            radar_range_button.show()
            bearing_lines_button.show()
            ship_inject_button.show()


        
        elif in_menu == True:

            self.channel_dropdown.hide()
            self.scuttle_button.hide()
            self.toggle_range_circle_button.hide()
            depth_dropdown.hide()
            launch_button.hide()
            arm_button.hide()
            auto_buoy_button.hide()
            auto_buoy_status_label.hide()
            controls_panel.hide()

            sonobuoy_dropdown.hide()
            torpedo_mode_label.hide()
            torpedo_mode_dropdown.hide()
            torpedo_freq_label.hide()
            torpedo_frequency_dropdown.hide()
            self.difar_display_button.hide()
            self.band_mode_button.hide()
            self.listen_button.hide()
            self.toggle_bearing_lines_button.hide()
            self.marker_type_dropdown.hide()
            self.marker_class_dropdown.hide()
            #IN MENU BUTTONS
            simulator_button.show()

            start_button.show()
            save_button.show()
            load_button.show()
            civilian_traffic_button.show()
            whale_traffic_button.show()
            map_mode_button.hide()
            radar_mode_button.hide()
            radar_orientation_button.hide()
            radar_range_button.hide()
            bearing_lines_button.hide()
            ship_inject_button.hide()







    def update_ui(self):
        channel_options = ["None"] + [option for option in sono_channel_array if option != "None"]
        if self.selected is not None and str(self.selected) not in channel_options:
            self.selected = None

        for e in (
            getattr(self, "channel_dropdown", None),
            getattr(self, "scuttle_button", None),
            getattr(self, "toggle_range_circle_button", None),
            getattr(self, "difar_display_button", None),
            getattr(self, "band_mode_button", None),
            getattr(self, "listen_button", None),
            getattr(self, "toggle_bearing_lines_button", None),
            getattr(self, "marker_type_dropdown", None),
            getattr(self, "marker_class_dropdown", None)
            ):
            
            if e is not None:
                e.kill()

        if self.selected != None:
            self.channel_dropdown = pygame_gui.elements.UIDropDownMenu(
                    channel_options,
                    starting_option=str(self.selected),
                    relative_rect=scale_rect(pygame.Rect((self.topleft[0], self.topleft[1]+1), (70, 25)),screen))
                
        if self.selected == None:
            self.channel_dropdown = pygame_gui.elements.UIDropDownMenu(
            channel_options,
            starting_option="None",
            relative_rect=scale_rect(pygame.Rect((self.topleft[0], self.topleft[1]+1), (72, 25)),screen))
        if self.topleft[1] <= 0:
            force_dropdown_down(self.channel_dropdown, 220)
        
        
        self.scuttle_button = pygame_gui.elements.UIButton(
            scale_rect(pygame.Rect((self.topleft[0]+398, self.topleft[1]+1), (78, 25)),screen),
            "SCUTTLE"
        )

        self.toggle_range_circle_button = pygame_gui.elements.UIButton(
            scale_rect(pygame.Rect((self.topleft[0]+76, self.topleft[1]+1), (22, 25)),screen),
            "O"
        )
        self.difar_display_button = pygame_gui.elements.UIButton(
            scale_rect(pygame.Rect((self.topleft[0]+102, self.topleft[1]+1), (58, 25)),screen),
            "SPEC" if self.display_mode == "SPECTROGRAM" else "AZI"
        )
        self.listen_button = pygame_gui.elements.UIButton(
            scale_rect(pygame.Rect((self.topleft[0]+164, self.topleft[1]+1), (58, 25)),screen),
            "L"
        )
        self.band_mode_button = pygame_gui.elements.UIButton(
            scale_rect(pygame.Rect((self.topleft[0], self.topleft[1]+29), (128, 24)),screen),
            "BROADBAND" if self.band_mode == "BROADBAND" else "NARROWBAND"
        )
        self.toggle_bearing_lines_button = pygame_gui.elements.UIButton(
            scale_rect(pygame.Rect((self.topleft[0]+134, self.topleft[1]+29), (68, 24)),screen),
            "LINES"
        )
        marker_class_options = spectrogram_marker_class_list.get(self.marker_type, ["None"])
        if self.marker_class not in marker_class_options:
            self.marker_class = marker_class_options[0]
        self.marker_type_dropdown = pygame_gui.elements.UIDropDownMenu(
            spectrogram_marker_type_list,
            starting_option=self.marker_type,
            relative_rect=scale_rect(pygame.Rect((self.topleft[0]+208, self.topleft[1]+29), (132, 24)),screen)
        )
        self.marker_class_dropdown = pygame_gui.elements.UIDropDownMenu(
            marker_class_options,
            starting_option=self.marker_class,
            relative_rect=scale_rect(pygame.Rect((self.topleft[0]+348, self.topleft[1]+29), (128, 24)),screen)
        )
        if self.topleft[1] <= 0:
            force_dropdown_down(self.marker_type_dropdown, 220)
            force_dropdown_down(self.marker_class_dropdown, 220)
        self.sync_difar_display_button_style()
        self.sync_band_mode_button_style()
        self.sync_bearing_lines_button_style()
        self.sync_listen_button_style()
        # Register the channel dropdown
        register_ui_element(self.channel_dropdown, scale_rect(pygame.Rect((self.topleft[0], self.topleft[1]+1), (72, 25)),screen))
        #Register the range ring button
        register_ui_element(self.toggle_range_circle_button , scale_rect(pygame.Rect((self.topleft[0]+76, self.topleft[1]+1), (22, 25)),screen))
        register_ui_element(self.difar_display_button , scale_rect(pygame.Rect((self.topleft[0]+102, self.topleft[1]+1), (58, 25)),screen))
        register_ui_element(self.listen_button, scale_rect(pygame.Rect((self.topleft[0]+164, self.topleft[1]+1), (58, 25)),screen))
        register_ui_element(self.band_mode_button, scale_rect(pygame.Rect((self.topleft[0], self.topleft[1]+29), (128, 24)),screen))
        register_ui_element(self.toggle_bearing_lines_button, scale_rect(pygame.Rect((self.topleft[0]+134, self.topleft[1]+29), (68, 24)),screen))
        register_ui_element(self.marker_type_dropdown, scale_rect(pygame.Rect((self.topleft[0]+208, self.topleft[1]+29), (132, 24)),screen))
        register_ui_element(self.marker_class_dropdown, scale_rect(pygame.Rect((self.topleft[0]+348, self.topleft[1]+29), (128, 24)),screen))
        # Register the SCUTTLE button
        register_ui_element(self.scuttle_button, scale_rect(pygame.Rect((self.topleft[0]+398, self.topleft[1]+1), (78, 25)),screen))
        self._last_in_menu = None


    def sync_difar_display_button_style(self):
        if not hasattr(self, "difar_display_button"):
            return
        if self.display_mode == "AZIGRAM":
            self.difar_display_button.set_text("AZI")
            new_colour = pygame.Color("#99C979")
        else:
            self.difar_display_button.set_text("SPEC")
            new_colour = pygame.Color("#4d6fa8")
        self.difar_display_button.colours["normal_bg"] = new_colour
        self.difar_display_button.colours["hovered_bg"] = new_colour
        self.difar_display_button.colours["active_bg"] = new_colour
        self.difar_display_button.rebuild()


    def sync_band_mode_button_style(self):
        if not hasattr(self, "band_mode_button"):
            return
        narrow = self.band_mode == "NARROWBAND"
        self.band_mode_button.set_text("NARROWBAND" if narrow else "BROADBAND")
        new_colour = pygame.Color("#99C979") if narrow else pygame.Color("#4d6fa8")
        self.band_mode_button.colours["normal_bg"] = new_colour
        self.band_mode_button.colours["hovered_bg"] = new_colour
        self.band_mode_button.colours["active_bg"] = new_colour
        self.band_mode_button.rebuild()


    def current_frequency_window(self):
        max_freq = fs / 2
        if self.band_mode != "NARROWBAND":
            return None
        if self.narrowband_drag_start_freq is not None and self.narrowband_drag_current_freq is not None:
            low = min(self.narrowband_drag_start_freq, self.narrowband_drag_current_freq)
            high = max(self.narrowband_drag_start_freq, self.narrowband_drag_current_freq)
            return (max(0.0, low), min(max_freq, max(high, low + 25.0)))
        if self.narrowband_window is not None:
            low, high = self.narrowband_window
            return (max(0.0, low), min(max_freq, max(high, low + 25.0)))
        return (0.0, min(max_freq, 1000.0))


    def plot_rect(self):
        return pygame.Rect(self.topleft[0] + 15, self.topleft[1] + 64, 450, 156)


    def screen_pos_to_plot_freq(self, screen_pos):
        internal_pos = internal_mouse_pos(screen_pos)
        internal_x = internal_pos.x
        internal_y = internal_pos.y
        spec_rect = self.plot_rect()
        if not spec_rect.collidepoint(internal_x, internal_y):
            return None
        rel_x = max(0, min(internal_x - spec_rect.left, spec_rect.width - 1))
        window = self.current_frequency_window() if self.band_mode == "NARROWBAND" else None
        if window is None:
            low, high = 0.0, fs / 2
        else:
            low, high = window
        return low + (rel_x / spec_rect.width) * (high - low)


    def full_freq_to_plot_x(self, freq, spec_rect):
        max_freq = fs / 2
        return int(spec_rect.left + (max(0.0, min(float(freq), max_freq)) / max_freq) * spec_rect.width)


    def narrowband_handle_rect(self):
        spec_rect = self.plot_rect()
        return pygame.Rect(spec_rect.left, spec_rect.bottom, spec_rect.width, 36)


    def screen_pos_to_narrowband_handle_freq(self, screen_pos):
        internal_pos = internal_mouse_pos(screen_pos)
        internal_x = internal_pos.x
        internal_y = internal_pos.y
        handle_rect = self.narrowband_handle_rect()
        if not handle_rect.collidepoint(internal_x, internal_y):
            return None
        rel_x = max(0, min(internal_x - handle_rect.left, handle_rect.width - 1))
        return (rel_x / handle_rect.width) * (fs / 2)


    def handle_narrowband_drag_start(self, screen_pos):
        freq = self.screen_pos_to_narrowband_handle_freq(screen_pos)
        if freq is None:
            return False
        self.narrowband_drag_base_window = (0.0, fs / 2)
        self.narrowband_drag_start_freq = freq
        self.narrowband_drag_current_freq = freq
        return True


    def handle_narrowband_drag_motion(self, screen_pos):
        if self.narrowband_drag_start_freq is None:
            return False
        internal_pos = internal_mouse_pos(screen_pos)
        internal_x = internal_pos.x
        handle_rect = self.narrowband_handle_rect()
        internal_x = max(handle_rect.left, min(internal_x, handle_rect.right))
        rel_x = internal_x - handle_rect.left
        self.narrowband_drag_current_freq = (rel_x / handle_rect.width) * (fs / 2)
        return True


    def handle_narrowband_drag_end(self, screen_pos):
        if self.narrowband_drag_start_freq is None:
            return False
        self.handle_narrowband_drag_motion(screen_pos)
        low = min(self.narrowband_drag_start_freq, self.narrowband_drag_current_freq)
        high = max(self.narrowband_drag_start_freq, self.narrowband_drag_current_freq)
        max_freq = fs / 2
        if high - low < 25.0:
            center = (low + high) * 0.5
            low = center - 125.0
            high = center + 125.0
        low = max(0.0, min(low, max_freq - 25.0))
        high = min(max_freq, max(high, low + 25.0))
        self.narrowband_window = (low, high)
        self.narrowband_drag_start_freq = None
        self.narrowband_drag_current_freq = None
        self.narrowband_drag_base_window = None
        return True


    def draw_narrowband_boundaries(self, spec_rect):
        if self.narrowband_drag_start_freq is not None and self.narrowband_drag_current_freq is not None:
            window = (min(self.narrowband_drag_start_freq, self.narrowband_drag_current_freq), max(self.narrowband_drag_start_freq, self.narrowband_drag_current_freq))
        elif self.narrowband_window is not None:
            window = self.narrowband_window
        else:
            window = (0.0, min(fs / 2, 1000.0))
        colour = (255, 230, 90)
        muted = (90, 110, 115)
        label_font = self.marker_font
        handle_y = spec_rect.bottom
        pygame.draw.line(data_Surface, muted, (spec_rect.left, handle_y), (spec_rect.right, handle_y), 1)
        for freq in window:
            x = self.full_freq_to_plot_x(freq, spec_rect)
            handle_points = [(x, handle_y - 8), (x - 6, handle_y + 2), (x + 6, handle_y + 2)]
            pygame.draw.polygon(data_Surface, colour, handle_points)
            pygame.draw.line(data_Surface, colour, (x, handle_y + 2), (x, handle_y + 12), 1)
            label = label_font.render(f"{freq:.0f}", False, colour)
            label_x = max(spec_rect.left, min(x - label.get_width() // 2, spec_rect.right - label.get_width()))
            data_Surface.blit(label, (label_x, handle_y + 13))
        if self.narrowband_drag_start_freq is not None and self.narrowband_drag_current_freq is not None:
            x0 = self.full_freq_to_plot_x(self.narrowband_drag_start_freq, spec_rect)
            x1 = self.full_freq_to_plot_x(self.narrowband_drag_current_freq, spec_rect)
            pygame.draw.line(data_Surface, colour, (x0, handle_y), (x1, handle_y), 3)


    def freq_to_plot_x(self, freq, spec_rect):
        window = self.current_frequency_window()
        max_freq = fs / 2
        if window is None:
            low, high = 0.0, max_freq
        else:
            low, high = window
        if freq < low or freq > high:
            return None
        return int(spec_rect.left + ((freq - low) / max(1.0, high - low)) * spec_rect.width)


    def sync_bearing_lines_button_style(self):
        if not hasattr(self, "toggle_bearing_lines_button"):
            return
        selected_sono = self.get_passive_sonobuoy_for_slot()
        line_state = self.bearing_lines_visible if selected_sono is None else getattr(selected_sono, "bearing_lines_visible", True)
        new_colour = pygame.Color("#99C979") if line_state else pygame.Color("#b13b3b")
        self.toggle_bearing_lines_button.set_text("LINES")
        self.toggle_bearing_lines_button.colours["normal_bg"] = new_colour
        self.toggle_bearing_lines_button.colours["hovered_bg"] = new_colour
        self.toggle_bearing_lines_button.colours["active_bg"] = new_colour
        self.toggle_bearing_lines_button.rebuild()


    def sync_listen_button_style(self):
        if not hasattr(self, "listen_button"):
            return
        active = listening_spectrogram_slot is self
        new_colour = pygame.Color("#99C979") if active else pygame.Color("#4d6fa8")
        self.listen_button.set_text("L")
        self.listen_button.colours["normal_bg"] = new_colour
        self.listen_button.colours["hovered_bg"] = new_colour
        self.listen_button.colours["active_bg"] = new_colour
        self.listen_button.rebuild()


    def get_passive_sonobuoy_for_slot(self):
        if self.selected in (None, "None") or not is_numeric_channel(self.selected):
            return None

        for sono in sono_array:
            if int(sono.channel) == int(self.selected):
                return sono

        return None


    def get_active_sonobuoy_for_slot(self):
        if self.selected in (None, "None") or not is_numeric_channel(self.selected):
            return None

        for active_sono in active_sono_array:
            if int(active_sono.channel) == int(self.selected):
                return active_sono

        return None


    def get_xbt_for_slot(self):
        return get_xbt_profile_for_selection(self.selected)


    def draw_xbt_profile_panel(self, xbt_profile, slot_rect):
        data_Surface.fill((5, 5, 10), rect=slot_rect)
        pygame.draw.rect(data_Surface, (92, 96, 98), slot_rect, 1)
        title_font = scaled_sys_font(14, bold=True)
        small_font = scaled_sys_font(11)
        title = title_font.render(f"{xbt_profile.label} SELECTED", False, (0, 220, 235))
        data_Surface.blit(title, (slot_rect.x + 12, slot_rect.y + 38))
        data_Surface.blit(small_font.render("Open the XBT tab below for full profile.", False, (220, 220, 220)), (slot_rect.x + 12, slot_rect.y + 62))
        data_Surface.blit(small_font.render("Graph is too large for this strip.", False, (140, 150, 152)), (slot_rect.x + 12, slot_rect.y + 78))


    def active_sonobuoy_status(self, active_sono):
        now = time.time()
        total_cycle = active_sono.sweep_time + active_sono.gap_time
        elapsed = (now - active_sono.sweep_start_time) % total_cycle

        if elapsed > active_sono.sweep_time:
            listen_elapsed = elapsed - active_sono.sweep_time
            return {
                "state": "LISTEN",
                "freq_hz": 0,
                "source_db": 0,
                "progress": listen_elapsed / active_sono.gap_time if active_sono.gap_time else 0,
                "cycle_elapsed": elapsed,
                "total_cycle": total_cycle
            }

        progress = elapsed / active_sono.sweep_time if active_sono.sweep_time else 0
        phase1_end = 0.45
        phase2_end = 0.90
        base_start = active_sono.start_khz
        base_end = active_sono.start_khz + active_sono.bandwidth
        final_start = base_end * 1.1
        final_end = final_start + active_sono.bandwidth
        source_db = active_sono.source_db

        if progress <= phase1_end:
            local = progress / phase1_end
            freq_khz = base_start + (base_end - base_start) * local
            source_db *= local
        elif progress <= phase2_end:
            freq_khz = base_end
        else:
            local = (progress - phase2_end) / (1.0 - phase2_end)
            freq_khz = final_start + (final_end - final_start) * local
            source_db *= (1.0 - local)

        return {
            "state": "TRANSMIT",
            "freq_hz": freq_khz * 1000,
            "source_db": source_db,
            "progress": progress,
            "cycle_elapsed": elapsed,
            "total_cycle": total_cycle
        }


    def marker_frequencies(self):
        if self.marker_type == "Marks Off":
            return []
        return spectrogram_marker_frequencies.get((self.marker_type, self.marker_class), [])


    def draw_frequency_markers(self, spec_rect):
        marker_freqs = self.marker_frequencies()
        if not marker_freqs:
            return

        marker_colour = (255, 190, 65)
        label_colour = (255, 220, 125)
        tick_top = spec_rect.bottom + 2
        tick_bottom = spec_rect.bottom + 12
        for freq in marker_freqs:
            x = self.freq_to_plot_x(freq, spec_rect)
            if x is None:
                continue
            pygame.draw.line(data_Surface, marker_colour, (x, tick_top), (x, tick_bottom), 1)
            pygame.draw.line(data_Surface, label_colour, (x - 4, tick_top), (x + 4, tick_top), 1)
            label = self.marker_font.render(f"{int(freq)}", False, label_colour)
            label_x = max(spec_rect.left, min(x - label.get_width() // 2, spec_rect.right - label.get_width()))
            data_Surface.blit(label, (label_x, tick_bottom + 12))


    def nearest_contact_range_nm(self, active_sono):
        nearest = None
        active_lat, active_lon = pix_to_latlong(active_sono.x, active_sono.y)

        for contact in contacts:
            if any(tone.label == "FMCW" for tone in getattr(contact, "tones", [])):
                continue

            distance = haversine(active_lat, active_lon, contact.contact_lat, contact.contact_long)
            if nearest is None or distance < nearest:
                nearest = distance

        return nearest


    def draw_active_sonobuoy_info(self, active_sono, slot_rect):
        status = self.active_sonobuoy_status(active_sono)
        lat, lon = pix_to_latlong(active_sono.x, active_sono.y)
        nearest = self.nearest_contact_range_nm(active_sono)
        nearest_text = f"{nearest:.1f} NM" if nearest is not None else "NONE"
        clutter_level = bottom_clutter_db(max(0.2, nearest or 3.0), getattr(active_sono, "depth", 0), max(100.0, status["freq_hz"]))
        panel_colour = (153, 201, 121) if status["state"] == "TRANSMIT" else (92, 96, 98)
        small_font = scaled_sys_font(12)
        title_font = scaled_sys_font(14, bold=True)

        pygame.draw.rect(data_Surface, panel_colour, slot_rect, 1)
        header = title_font.render(f"ACTIVE DICASS  CH {active_sono.channel}", False, panel_colour)
        data_Surface.blit(header, (slot_rect.x + 12, slot_rect.y + 30))

        rows = [
            f"State: {status['state']}",
            f"Sweep: {active_sono.start_khz:.1f}-{active_sono.end_khz:.1f} kHz",
            f"Now: {status['freq_hz']:.0f} Hz  SL: {status['source_db']:.0f} dB",
            f"Cycle: {status['cycle_elapsed']:.1f}/{status['total_cycle']:.1f}s",
            f"Nearest contact: {nearest_text}",
            f"Layer: {acoustic_layer_depth():.0f} m  Bottom clutter: {clutter_level:.1f} dB",
            f"Depth: {active_sono.depth} ft",
            f"Lat {lat:.4f}  Lon {lon:.4f}"
        ]

        for i, text in enumerate(rows):
            label = small_font.render(text, False, (235, 235, 235))
            data_Surface.blit(label, (slot_rect.x + 12, slot_rect.y + 50 + i * 12))

        bar_rect = pygame.Rect(slot_rect.x + 220, slot_rect.y + 32, 230, 8)
        pygame.draw.rect(data_Surface, (35, 40, 42), bar_rect)
        fill_rect = bar_rect.copy()
        fill_rect.width = int(bar_rect.width * max(0, min(1, status["progress"])))
        pygame.draw.rect(data_Surface, panel_colour, fill_rect)
        pygame.draw.rect(data_Surface, (92, 96, 98), bar_rect, 1)



    def handle_azigram_right_click(self, screen_pos):
        if self.display_mode != "AZIGRAM" or self.selected in (None, "None") or not is_numeric_channel(self.selected):
            return False
        spectro_to_use = None
        for spectro in spectro_array:
            if int(spectro.sono.channel) == int(self.selected):
                spectro_to_use = spectro
                break
        if spectro_to_use is None:
            return False
        internal_pos = internal_mouse_pos(screen_pos)
        internal_x = internal_pos.x
        internal_y = internal_pos.y
        spec_rect = self.plot_rect()
        if not spec_rect.collidepoint(internal_x, internal_y):
            return False
        rel_x = max(0, min(internal_x - spec_rect.left, spec_rect.width - 1))
        freq_window = self.current_frequency_window()
        if freq_window is None:
            low_freq, high_freq = 0.0, fs / 2
        else:
            low_freq, high_freq = freq_window
        clicked_freq = low_freq + (rel_x / spec_rect.width) * (high_freq - low_freq)
        nearest = None
        nearest_distance = float("inf")
        for detection in getattr(spectro_to_use.sono, "detections", []):
            distance = abs(float(detection.get("freq", 0.0)) - clicked_freq)
            if distance < nearest_distance:
                nearest = detection
                nearest_distance = distance
        if nearest is None or nearest_distance > max(40.0, (high_freq - low_freq) * 0.08):
            return True
        add_manual_azigram_bearing_line(spectro_to_use.sono, nearest)
        return True


    def draw(self):
        if not self.active:
            return

        slot_rect = pygame.Rect(self.topleft[0], self.topleft[1], 480, 270)
        data_Surface.fill((5, 5, 10), rect=slot_rect)
        pygame.draw.rect(data_Surface, (92, 96, 98), slot_rect, 1)

        active_sono = self.get_active_sonobuoy_for_slot()
        if active_sono is not None:
            self.draw_active_sonobuoy_info(active_sono, slot_rect)
            return

        # --- Draw spectrogram if channel selected ---
        spectro_to_use = None
        if self.selected not in (None, "None"):
            for spectro in spectro_array:
                if int(spectro.sono.channel) == int(self.selected):
                    data_Surface.blit(spectro.render_surface(self.display_mode, self.current_frequency_window()), (self.topleft[0], self.topleft[1]))
                    spectro_to_use = spectro
                    break

        if spectro_to_use is None:
            return

        # --- Spectrogram plot rectangle inside slot ---
        spec_x_offset = 15
        spec_y_offset = 64
        spec_width = 450
        spec_height = 156
        spec_rect = pygame.Rect(
            self.topleft[0] + spec_x_offset,
            self.topleft[1] + spec_y_offset,
            spec_width,
            spec_height
        )
        self.draw_frequency_markers(spec_rect)
        self.draw_narrowband_boundaries(spec_rect)

        # --- Mouse position ---
        mouse_x, mouse_y = pygame.mouse.get_pos()

        # --- Convert mouse to INTERNAL coords ---
        internal_mouse = internal_mouse_pos((mouse_x, mouse_y))
        internal_mouse_x = internal_mouse.x
        internal_mouse_y = internal_mouse.y

        # spec_rect is already in the app's internal coordinate space.
        internal_rect = spec_rect

        if internal_rect.collidepoint(internal_mouse_x, internal_mouse_y):

                # --- Relative coords ---
                rel_x = internal_mouse_x - internal_rect.left
                rel_y = internal_mouse_y - internal_rect.top

                rel_x = max(0, min(rel_x, internal_rect.width - 1))
                rel_y = max(0, min(rel_y, internal_rect.height - 1))

                # --- Data conversion ---
                freq_window = self.current_frequency_window()
                if freq_window is None:
                    low_freq, high_freq = 0.0, fs / 2
                else:
                    low_freq, high_freq = freq_window
                x_value_at_mouse = low_freq + (rel_x / internal_rect.width) * (high_freq - low_freq)
                seconds_ago = (1 - rel_y / internal_rect.height) * self.time_window_sec

                # --- Map to spectrogram/azigram plot surface ---
                plot_surface = spectro_to_use.azi_surface if self.display_mode == "AZIGRAM" else spectro_to_use.gram_surface
                surf_w, surf_h = plot_surface.get_size()

                if freq_window is None:
                    surf_x = int((rel_x / internal_rect.width) * surf_w)
                else:
                    surf_x = int((x_value_at_mouse / (fs / 2)) * surf_w)
                surf_y = int((rel_y / internal_rect.height) * surf_h)

                surf_x = max(0, min(surf_x, surf_w - 1))
                surf_y = max(0, min(surf_y, surf_h - 1))

                # --- Read pixel ---
                pixel_color = plot_surface.get_at((surf_x, surf_y))


                display_amplitude = sum(pixel_color[:3]) / (3 * 255)
                dbfs_value = 20 * math.log10(max(display_amplitude, 1e-6))
                # --- Crosshair and readout ---
                cursor_colour = (255, 230, 90)
                pygame.draw.line(
                    data_Surface,
                    cursor_colour,
                    (int(internal_mouse_x), internal_rect.top),
                    (int(internal_mouse_x), internal_rect.bottom),
                    1
                )
                pygame.draw.line(
                    data_Surface,
                    cursor_colour,
                    (internal_rect.left, int(internal_mouse_y)),
                    (internal_rect.right, int(internal_mouse_y)),
                    1
                )

                amplitude_text = f"{display_amplitude:.2f} FS  {dbfs_value:.1f} dBFS"
                if self.display_mode == "AZIGRAM":
                    bearing_text = "--"
                    nearest = None
                    nearest_distance = 999
                    for detection in getattr(spectro_to_use.sono, "detections", []):
                        detection_x_abs = self.freq_to_plot_x(detection["freq"], internal_rect)
                        if detection_x_abs is None:
                            continue
                        detection_x = detection_x_abs - internal_rect.left
                        distance = abs(rel_x - detection_x)
                        if distance < nearest_distance:
                            nearest = detection
                            nearest_distance = distance
                    if nearest is not None and nearest_distance <= 18:
                        bearing_text = f"{nearest['bearing']:.0f} deg"
                    tooltip_text = f"{x_value_at_mouse:.0f} Hz   {amplitude_text}   BRG {bearing_text}   {seconds_ago:.2f}s"
                else:
                    tooltip_text = f"{x_value_at_mouse:.0f} Hz   {amplitude_text}   {seconds_ago:.2f}s"
                tooltip_surf = spectro_to_use.tooltip_font.render(tooltip_text, True, cursor_colour)
                tooltip_bg = pygame.Surface(
                    (tooltip_surf.get_width() + 8, tooltip_surf.get_height() + 6),
                    pygame.SRCALPHA
                )
                tooltip_bg.fill((0, 0, 0, 190))
                tooltip_bg.blit(tooltip_surf, (4, 3))

                tooltip_x = int(min(
                    max(internal_mouse_x + 10, slot_rect.left + 4),
                    slot_rect.right - tooltip_bg.get_width() - 4
                ))
                tooltip_y = int(max(
                    slot_rect.top + 28,
                    internal_mouse_y - tooltip_bg.get_height() - 10
                ))
                data_Surface.blit(tooltip_bg, (tooltip_x, tooltip_y))
spectrogram_slot_position_array = [(0,0),(480,0),(0,270),(480,270)]    
spectrogram_slot_array = []
for i,spectrogram_slot_position in enumerate(spectrogram_slot_position_array):
    spectrogram_slot_array.append(SpectrogramSlot(spectrogram_slot_position,i))



        





depth = 90





class Submarine(pygame.sprite.Sprite):
    def __init__(self, sub_pos, prop_noise, pump_noise, cav_noise, prop_noise_db, pump_noise_db, cav_noise_db, depth, source_noise):
        self.image = sub_surface
        self.rect = self.image.get_rect(center=latlong_to_pix(submarine_latitude,submarine_longitude))
        self.x = sub_pos.x
        self.y = sub_pos.y
        self.prop_noise = prop_noise
        self.pump_noise = pump_noise
        self.cav_noise = cav_noise
        self.prop_noise_db = prop_noise_db
        self.pump_noise_db = pump_noise_db
        self.cav_noise_db = cav_noise_db
        self.depth = depth
        self.source_noise = source_noise
        self.angle = 0                # current heading angle in degrees
        self.turn_speed = 2 






class Tone:
    """A tonal source component emitted by a contact, with optional harmonics."""

    def __init__(self, freq, db, label,
                 harmonics=1, harmonic_drop=6):
        
        self.freq = freq
        self.db = db
        self.label = label
        self.received_db = {}
        self.received_freq = {}
        self.harmonics = harmonics          # number of harmonics
        self.harmonic_drop = harmonic_drop  # dB drop per harmonic

    def set_received_db_and_freq(self,sono_channel,received_db,received_freq):
        self.received_db[str(sono_channel)] = received_db
        self.received_freq[str(sono_channel)] = received_freq
    

    def get_components(self,sono_channel):
        components = []
        if str(sono_channel) not in self.received_db:
            return components

        for n in range(1, self.harmonics + 1):

            # Small RPM wobble
            wobble = random.uniform(0.998, 1.002)

            base_freq = self.received_freq.get(str(sono_channel), self.freq)
            freq = base_freq * n * wobble
            db = self.received_db[str(sono_channel)] - (self.harmonic_drop * (n - 1))

            components.append((freq, db))

        return components
    
    def calc_received_db(self,sono_latlong, contact_lat,contact_long):
        dist = haversine(contact_lat,contact_long,pix_to_latlong(sono_latlong[0],sono_latlong[1])[0],pix_to_latlong(sono_latlong[0],sono_latlong[1])[1])

        return self.db - difar_transmission_loss(dist, self.freq)
        

        
    def __repr__(self):
        return (f"Tone(freq={self.freq:.2f} Hz, db={self.db}"
                f"label={self.label}, harmonics={self.harmonics}, "
                f"harmonic_drop={self.harmonic_drop},received_db ={self.received_db},received_freq ={self.received_freq})")


class WhaleCallTone(Tone):
    """Biological tonal call with a slow swept contour and pulsed amplitude."""

    def __init__(self, freq, db, label, sweep_hz=80, period_sec=7.0, phase=0.0,
                 duty=0.55, harmonics=1, harmonic_drop=7):
        super().__init__(freq, db, label, harmonics=harmonics, harmonic_drop=harmonic_drop)
        self.sweep_hz = sweep_hz
        self.period_sec = max(0.5, period_sec)
        self.phase = phase
        self.duty = max(0.1, min(1.0, duty))

    def get_components(self, sono_channel):
        components = []
        if str(sono_channel) not in self.received_db:
            return components

        now = time.time()
        cycle = ((now / self.period_sec) + self.phase) % 1.0
        if cycle > self.duty:
            return components

        call_progress = cycle / self.duty
        envelope = math.sin(math.pi * call_progress) ** 0.75
        sweep_shape = math.sin((call_progress * math.pi * 1.25) - (math.pi * 0.25))
        base_freq = self.received_freq.get(str(sono_channel), self.freq)
        swept_freq = max(15.0, base_freq + (self.sweep_hz * sweep_shape))
        pulsed_db = self.received_db[str(sono_channel)] - 18.0 + (10.0 * envelope)

        for n in range(1, self.harmonics + 1):
            wobble = random.uniform(0.995, 1.005)
            freq = swept_freq * n * wobble
            db = pulsed_db - (self.harmonic_drop * (n - 1))
            components.append((freq, db))

        return components


class Contact:
    """A moving object in the acoustic world, usually a submarine contact."""

    def __init__(self, name, tones, contact_lat, contact_long, speed, depth, bearing):

        self.name = name
        self.tones = tones
        self.contact_lat = contact_lat
        self.contact_long = contact_long
        self.speed = speed
        self.depth = depth
        self.bearing = bearing
        self.detected = False
        self.detecting_buoys = set()
        self.type_dropdown = None
        self.class_dropdown = None
        self.internal_type = "Sub-surface"
        self.internal_class = self.name
        self.classification_type = "Unknown"
        self.classification_class = "Unknown"
        self.identity_status = "P"
        self.country = "Unknown"
        self.operator_classified = False
        self.gaist_model_title = ""
        self.broadcasting = False
        self.bearing_lines_hidden = False
        self.ui_drawn = False
        self.trail = []
        self.base_tone_dbs = None
        self.behaviour_state = "NORMAL"
        self.behaviour_until = 0.0
        self.evading_torpedo = None
        self.last_behaviour_print = ""
        self.next_evasion_turn_time = 0.0
        self.desired_bearing = bearing
        self.original_speed = speed
        self.target_speed = speed
        self.target_depth = depth
        self.current_acoustic_reduction_db = 0.0
        self.target_acoustic_reduction_db = 0.0
        self.blade_count = None
        self.shaft_rate_hz = None
        self.blade_rate_hz = None
        self.team = "Neutral"
        self.route_waypoints = []
        self.shadow_target_name = ""
        self.shadow_distance_nm = 5.0
        self.route_index = 0
        self.route_active = False
        self.route_loop = False
        self.route_speed_kts = None
        self.route_started_at_utc = None
        self.route_status = "No route"
        # Convert lat/long to pixel position
        x, y = latlong_to_pix(contact_lat, contact_long)

        # FLOAT position (real physics position)
        self.pos = pygame.Vector2(x, y)

        # Rect for drawing and clicking
        self.contact_rect = pygame.Rect(0, 0, 32, 32)


        self.track_number = random.randint(1, 9999)
    def __iter__(self):
        return iter(self.tones)

    def __len__(self):
        return len(self.tones)

    def move(self, dt_seconds=1/60, nm_per_pix=2.6):

        pix_per_frame = (self.speed / nm_per_pix) * (dt_seconds / 3600.0)

        dx = math.sin(math.radians(self.bearing)) * pix_per_frame
        dy = -math.cos(math.radians(self.bearing)) * pix_per_frame

        # Update FLOAT position
        self.pos.x += dx
        self.pos.y += dy

        # Sync rect to float position
        self.contact_rect.center = (round(self.pos.x), round(self.pos.y))

        # Update lat/long from float position
        self.contact_lat, self.contact_long = pix_to_latlong(
            self.pos.x,
            self.pos.y
        )
    def set_bearing(self, new_bearing_deg):
        """Update the contact's bearing."""
        self.bearing = new_bearing_deg % 360  # keep within 0-359


    def __repr__(self):
        tone_str = "[" + ", ".join(repr(t) for t in self.tones) + "]"
        return f"Contact({self.name}, lat={self.contact_lat:.2f}, lon={self.contact_long:.2f}, tones={tone_str})"
contacts = []


def contact_is_submarine(contact):
    return (
        getattr(contact, "internal_type", "Sub-surface") == "Sub-surface" and
        not is_dicass_ping_contact(contact) and
        hasattr(contact, "tones")
    )


def ensure_contact_base_tones(contact):
    if getattr(contact, "base_tone_dbs", None) is None:
        contact.base_tone_dbs = [tone.db for tone in getattr(contact, "tones", [])]
    if len(contact.base_tone_dbs) != len(getattr(contact, "tones", [])):
        contact.base_tone_dbs = [tone.db for tone in getattr(contact, "tones", [])]


def set_contact_acoustic_scale(contact, reduction_db):
    ensure_contact_base_tones(contact)
    for tone, base_db in zip(contact.tones, contact.base_tone_dbs):
        tone.db = max(25.0, base_db - reduction_db)


def print_sub_behaviour(contact, state_key, message):
    if getattr(contact, "last_behaviour_print", "") == state_key:
        return
    contact.last_behaviour_print = state_key
    print(f"[SUB {getattr(contact, 'track_number', '----')} {contact.name}] {message}")


def bearing_from_world_to_world(start_world, end_world):
    start_lat, start_lon = pix_to_latlong(start_world.x, start_world.y)
    end_lat, end_lon = pix_to_latlong(end_world.x, end_world.y)
    return haversine_bearing(start_lat, start_lon, end_lat, end_lon)


def approach_value(current, target, max_delta):
    if current < target:
        return min(target, current + max_delta)
    return max(target, current - max_delta)


def approach_bearing(current, target, max_delta):
    error = (target - current + 180) % 360 - 180
    return (current + max(-max_delta, min(max_delta, error))) % 360


def set_sub_reaction_targets(contact, state, reduction_db, target_speed=None, target_depth=None, desired_bearing=None):
    contact.behaviour_state = state
    contact.target_acoustic_reduction_db = reduction_db
    if target_speed is not None:
        contact.target_speed = target_speed
    if target_depth is not None:
        contact.target_depth = target_depth
    if desired_bearing is not None:
        contact.desired_bearing = desired_bearing % 360


def apply_sub_reaction_dynamics(contact, dt_seconds):
    reduction_rate_db_s = 2.0
    speed_rate_kts_s = 1.2
    depth_rate_ft_s = 70.0
    turn_rate_deg_s = 5.0

    contact.current_acoustic_reduction_db = approach_value(
        getattr(contact, "current_acoustic_reduction_db", 0.0),
        getattr(contact, "target_acoustic_reduction_db", 0.0),
        reduction_rate_db_s * dt_seconds
    )
    set_contact_acoustic_scale(contact, contact.current_acoustic_reduction_db)

    contact.speed = approach_value(
        float(getattr(contact, "speed", 0.0) or 0.0),
        float(getattr(contact, "target_speed", getattr(contact, "speed", 0.0)) or 0.0),
        speed_rate_kts_s * dt_seconds
    )
    contact.depth = approach_value(
        float(getattr(contact, "depth", 0.0) or 0.0),
        float(getattr(contact, "target_depth", getattr(contact, "depth", 0.0)) or 0.0),
        depth_rate_ft_s * dt_seconds
    )
    contact.bearing = approach_bearing(
        float(getattr(contact, "bearing", 0.0) or 0.0),
        float(getattr(contact, "desired_bearing", getattr(contact, "bearing", 0.0)) or 0.0),
        turn_rate_deg_s * dt_seconds
    )


def update_submarine_reactions(dt_seconds):
    now = time.time()

    for contact in contacts:
        if not contact_is_submarine(contact):
            continue
        if str(getattr(contact, "shadow_target_name", "") or "").strip():
            continue

        ensure_contact_base_tones(contact)
        contact_pos = pygame.Vector2(latlong_to_pix(contact.contact_lat, contact.contact_long))
        contact.original_speed = max(float(getattr(contact, "original_speed", getattr(contact, "speed", 0)) or 0), 0.0)

        nearest_passive_buoy_nm = None
        for sono in sono_array:
            distance_nm = world_distance_nm(contact_pos, pygame.Vector2(sono.x, sono.y))
            if nearest_passive_buoy_nm is None or distance_nm < nearest_passive_buoy_nm:
                nearest_passive_buoy_nm = distance_nm

        nearest_dicass_nm = None
        active_dicass_close = False
        for active_sono in active_sono_array:
            active_pos = pygame.Vector2(active_sono.x, active_sono.y)
            distance_nm = world_distance_nm(contact_pos, active_pos)
            if nearest_dicass_nm is None or distance_nm < nearest_dicass_nm:
                nearest_dicass_nm = distance_nm
            if distance_nm <= 8.0:
                active_dicass_close = True

        nearest_torpedo = None
        nearest_torpedo_nm = None
        for torpedo in torp_array:
            if getattr(torpedo, "finished", False) or getattr(torpedo, "detonated", False):
                continue
            distance_nm = world_distance_nm(contact_pos, torpedo.pos)
            if nearest_torpedo_nm is None or distance_nm < nearest_torpedo_nm:
                nearest_torpedo_nm = distance_nm
                nearest_torpedo = torpedo

        threatened_by_torpedo = nearest_torpedo is not None and nearest_torpedo_nm <= 2.5
        threatened_by_dicass = active_dicass_close
        disturbed_by_difar = nearest_passive_buoy_nm is not None and nearest_passive_buoy_nm <= 3.0

        if threatened_by_torpedo:
            contact.behaviour_until = now + 35.0
            contact.evading_torpedo = nearest_torpedo
            if now >= getattr(contact, "next_evasion_turn_time", 0.0):
                away_bearing = bearing_from_world_to_world(nearest_torpedo.pos, contact_pos)
                contact.desired_bearing = (away_bearing + random.uniform(-35.0, 35.0)) % 360
                contact.next_evasion_turn_time = now + random.uniform(4.0, 7.0)
            set_sub_reaction_targets(
                contact,
                "EVADING",
                5.0,
                target_speed=max(float(getattr(contact, "target_speed", 0) or 0), 14.0),
                target_depth=min(10000, float(getattr(contact, "target_depth", contact.depth) or 0) + 900.0),
                desired_bearing=getattr(contact, "desired_bearing", contact.bearing)
            )
            apply_sub_reaction_dynamics(contact, dt_seconds)
            print_sub_behaviour(
                contact,
                "TORPEDO_EVADING",
                f"TORPEDO EVASION: range {nearest_torpedo_nm:.2f} NM, turning toward {contact.desired_bearing:03.0f}, target speed {contact.target_speed:.1f} kt, quieting toward -5 dB"
            )
            continue

        if threatened_by_dicass:
            contact.behaviour_until = now + 45.0
            set_sub_reaction_targets(
                contact,
                "SILENT",
                4.0,
                target_speed=min(float(getattr(contact, "target_speed", getattr(contact, "speed", 0)) or 0), 4.0)
            )
            apply_sub_reaction_dynamics(contact, dt_seconds)
            print_sub_behaviour(
                contact,
                "DICASS_SILENT",
                f"DICASS NEARBY: silent running, slowing toward {contact.target_speed:.1f} kt, quieting toward -4 dB"
            )
            continue

        if disturbed_by_difar:
            contact.behaviour_until = max(getattr(contact, "behaviour_until", 0.0), now + 18.0)
            set_sub_reaction_targets(
                contact,
                "CAUTIOUS",
                2.0,
                target_speed=min(float(getattr(contact, "target_speed", getattr(contact, "speed", 0)) or 0), 6.0)
            )
            apply_sub_reaction_dynamics(contact, dt_seconds)
            print_sub_behaviour(
                contact,
                "DIFAR_CAUTIOUS",
                f"DIFAR SPLASH/BUOY CLOSE: nearest buoy {nearest_passive_buoy_nm:.2f} NM, slowing toward {contact.target_speed:.1f} kt, quieting toward -2 dB"
            )
            continue

        if now <= getattr(contact, "behaviour_until", 0.0):
            if getattr(contact, "behaviour_state", "NORMAL") == "EVADING":
                set_sub_reaction_targets(contact, "EVADING", 4.0)
            elif getattr(contact, "behaviour_state", "NORMAL") == "SILENT":
                set_sub_reaction_targets(contact, "SILENT", 3.0)
            else:
                set_sub_reaction_targets(contact, "CAUTIOUS", 1.5)
            apply_sub_reaction_dynamics(contact, dt_seconds)
            continue

        if getattr(contact, "behaviour_state", "NORMAL") != "NORMAL":
            contact.evading_torpedo = None
            set_sub_reaction_targets(
                contact,
                "NORMAL",
                0.0,
                target_speed=getattr(contact, "original_speed", contact.speed),
                target_depth=getattr(contact, "target_depth", contact.depth),
                desired_bearing=getattr(contact, "bearing", 0.0)
            )
            apply_sub_reaction_dynamics(contact, dt_seconds)
            print_sub_behaviour(contact, "NORMAL", "resuming normal acoustic profile")
        else:
            set_sub_reaction_targets(contact, "NORMAL", 0.0, target_speed=getattr(contact, "original_speed", contact.speed))
            apply_sub_reaction_dynamics(contact, dt_seconds)


def scaled_tone(freq, db, speed, reference_speed, label, harmonics=1, harmonic_drop=6, frequency_speed_strength=0.0):
    raw_speed_scale = max(0.65, min(1.45, speed / reference_speed))
    speed_scale = 1.0 + ((raw_speed_scale - 1.0) * frequency_speed_strength)
    freq_jitter = random.uniform(0.96, 1.04)
    db_jitter = random.uniform(-2.0, 2.0)
    return Tone(
        freq=freq * speed_scale * freq_jitter,
        db=db + db_jitter,
        label=label,
        harmonics=harmonics,
        harmonic_drop=harmonic_drop
    )


def propulsor_tones(blade_rate_hz, blade_count, db, speed, reference_speed, speed_strength=0.75):
    blade_count = max(1, int(blade_count))
    raw_speed_scale = max(0.65, min(1.45, float(speed or 0) / max(1.0, reference_speed)))
    speed_scale = 1.0 + ((raw_speed_scale - 1.0) * speed_strength)
    freq_jitter = random.uniform(0.985, 1.015)
    db_jitter = random.uniform(-1.5, 1.5)
    blade_rate = blade_rate_hz * speed_scale * freq_jitter
    shaft_rate = blade_rate / blade_count
    return [
        Tone(freq=shaft_rate, db=db - 14 + db_jitter, label="Shaft Rate", harmonics=min(3, blade_count), harmonic_drop=7),
        Tone(freq=blade_rate, db=db + db_jitter, label="Blade Rate", harmonics=2, harmonic_drop=8)
    ], shaft_rate, blade_rate


def set_propulsor_metadata(contact, blade_count, shaft_rate_hz, blade_rate_hz):
    contact.blade_count = int(blade_count)
    contact.shaft_rate_hz = float(shaft_rate_hz)
    contact.blade_rate_hz = float(blade_rate_hz)

BLADE_COUNT_BY_PROFILE = {
    "Fishing": 4,
    "Yacht": 3,
    "Tug": 4,
    "Ferry": 5,
    "Tanker": 5,
    "Cargo": 5,
    "Destroyer": 5,
    "Frigate": 5,
    "Carrier": 5,
    "Warship": 5,
    "Kilo": 7,
    "Akula": 7,
    "Delta IV": 7,
    "Borei": 7,
    "Yasen": 7,
    "Emitter": 4,
}


def apply_blade_count_metadata(contact, profile_name=None):
    profile_name = profile_name or getattr(contact, "acoustic_profile", None) or getattr(contact, "name", "")
    blade_count = BLADE_COUNT_BY_PROFILE.get(profile_name)
    if blade_count is None:
        return contact

    blade_tone = None
    for tone in getattr(contact, "tones", []):
        if tone.label in ("Propeller", "Blade Rate"):
            blade_tone = tone
            break
    if blade_tone is None:
        return contact

    blade_tone.label = "Blade Rate"
    blade_rate = max(0.1, float(blade_tone.freq))
    shaft_rate = blade_rate / max(1, int(blade_count))
    contact.tones = [tone for tone in contact.tones if tone.label != "Shaft Rate"]
    contact.tones.insert(
        0,
        Tone(
            freq=shaft_rate,
            db=max(25.0, float(blade_tone.db) - 14.0),
            label="Shaft Rate",
            harmonics=min(3, int(blade_count)),
            harmonic_drop=7
        )
    )
    set_propulsor_metadata(contact, blade_count, shaft_rate, blade_rate)
    return contact

def civilian_acoustic_profile_for_model(model_title, speed):
    title = (model_title or "").lower()

    if any(word in title for word in ("fishing", "trawler", "shen")):
        profile_name = "Fishing"
        tones = [
            scaled_tone(95, 94, speed, 10, "Diesel", 4, 5),
            scaled_tone(185, 91, speed, 10, "Propeller", 5, 6),
            scaled_tone(740, 78, speed, 10, "Generator", 2, 5)
        ]
    elif any(word in title for word in ("yacht", "motoryacht", "pleasure", "sail")):
        profile_name = "Yacht"
        tones = [
            scaled_tone(145, 84, speed, 16, "Propeller", 3, 7),
            scaled_tone(520, 74, speed, 16, "Generator", 2, 6),
            scaled_tone(1180, 68, speed, 16, "Aux Pump", 1, 6)
        ]
    elif any(word in title for word in ("tug", "asd", "supply", "psv")):
        profile_name = "Tug"
        tones = [
            scaled_tone(80, 102, speed, 12, "Diesel", 5, 5),
            scaled_tone(160, 100, speed, 12, "Propeller", 6, 5),
            scaled_tone(620, 86, speed, 12, "Hydraulic", 2, 6)
        ]
    elif any(word in title for word in ("ferry", "ro-ro", "roro", "passenger")):
        profile_name = "Ferry"
        tones = [
            scaled_tone(115, 99, speed, 18, "Diesel", 4, 5),
            scaled_tone(235, 96, speed, 18, "Propeller", 5, 6),
            scaled_tone(900, 82, speed, 18, "Generator", 2, 6)
        ]
    elif any(word in title for word in ("tanker", "lng", "lpg", "oil")):
        profile_name = "Tanker"
        tones = [
            scaled_tone(55, 108, speed, 14, "Slow Diesel", 5, 4),
            scaled_tone(110, 104, speed, 14, "Propeller", 6, 5),
            scaled_tone(410, 86, speed, 14, "Cargo Pump", 3, 6)
        ]
    else:
        profile_name = "Cargo"
        tones = [
            scaled_tone(70, 104, speed, 16, "Diesel", 5, 5),
            scaled_tone(140, 100, speed, 16, "Propeller", 6, 5),
            scaled_tone(480, 84, speed, 16, "Generator", 2, 6)
        ]

    if speed >= 18:
        tones.append(scaled_tone(2100, 78, speed, 20, "Flow Noise", 1, 6))

    return profile_name, tones


def apply_civilian_acoustic_profile(contact):
    if not getattr(contact, "gaist_model_title", ""):
        gaist_model_title_for_contact(contact)

    profile_name, tones = civilian_acoustic_profile_for_model(
        getattr(contact, "gaist_model_title", ""),
        float(getattr(contact, "speed", 0) or 0)
    )
    contact.acoustic_profile = profile_name
    contact.tones = tones
    apply_blade_count_metadata(contact, profile_name)
    return contact


def military_surface_acoustic_profile(contact_class, speed):
    contact_class = contact_class or "Unknown"
    if contact_class == "Destroyer":
        profile_name = "Destroyer"
        tones = [
            scaled_tone(120, 112, speed, 20, "Gas Turbine", 4, 5),
            scaled_tone(245, 108, speed, 20, "Propeller", 6, 5),
            scaled_tone(980, 92, speed, 20, "Gearbox", 3, 6),
            scaled_tone(1850, 84, speed, 22, "Pump", 2, 6)
        ]
    elif contact_class == "Frigate":
        profile_name = "Frigate"
        tones = [
            scaled_tone(105, 108, speed, 18, "Diesel Turbine", 4, 5),
            scaled_tone(215, 104, speed, 18, "Propeller", 5, 5),
            scaled_tone(820, 90, speed, 18, "Generator", 3, 6),
            scaled_tone(1650, 82, speed, 20, "Pump", 2, 6)
        ]
    elif contact_class == "Carrier":
        profile_name = "Carrier"
        tones = [
            scaled_tone(60, 118, speed, 18, "Main Machinery", 5, 4),
            scaled_tone(125, 114, speed, 18, "Propeller", 6, 5),
            scaled_tone(360, 101, speed, 18, "Reduction Gear", 4, 5),
            scaled_tone(720, 94, speed, 18, "Aux Machinery", 3, 6)
        ]
    else:
        profile_name = "Warship"
        tones = [
            scaled_tone(100, 106, speed, 18, "Machinery", 4, 5),
            scaled_tone(205, 102, speed, 18, "Propeller", 5, 5),
            scaled_tone(760, 88, speed, 18, "Generator", 2, 6)
        ]

    if speed >= 18:
        tones.append(scaled_tone(2400, 86, speed, 22, "Flow Noise", 1, 6))

    return profile_name, tones


def apply_surface_ship_acoustic_profile(contact):
    if getattr(contact, "internal_class", "") == "Civilian":
        return apply_civilian_acoustic_profile(contact)

    profile_name, tones = military_surface_acoustic_profile(
        getattr(contact, "internal_class", "Unknown"),
        float(getattr(contact, "speed", 0) or 0)
    )
    contact.acoustic_profile = profile_name
    contact.tones = tones
    apply_blade_count_metadata(contact, profile_name)
    return contact


def fresh_msfs_aircraft_latlon(max_wait_sec=1.25):
    global lat, long, hdg

    if not ensure_msfs_connection():
        return None

    aq_obj = globals().get("aq")
    if aq_obj is None:
        return None

    deadline = time.time() + max_wait_sec
    while time.time() <= deadline:
        sim_lat = aq_obj.get("PLANE_LATITUDE")
        sim_long = aq_obj.get("PLANE_LONGITUDE")
        sim_hdg = aq_obj.get("PLANE_HEADING_DEGREES_TRUE")
        try:
            if (
                sim_lat is not None and
                sim_long is not None and
                math.isfinite(float(sim_lat)) and
                math.isfinite(float(sim_long))
            ):
                lat = sim_lat
                long = sim_long
                if sim_hdg is not None:
                    hdg = math.degrees(sim_hdg)
                return float(lat), float(long)
        except (TypeError, ValueError):
            pass
        time.sleep(0.05)

    return None


def current_ownship_latlon():
    if xplane == 0:
        msfs_pos = fresh_msfs_aircraft_latlon()
        if msfs_pos is None:
            print("Civilian traffic skipped: MSFS returned no fresh aircraft lat/lon")
        return msfs_pos

    if lat is not None and long is not None:
        if abs(float(lat) - 1.0) < 0.0001 and abs(float(long) - 1.0) < 0.0001:
            msfs_pos = fresh_msfs_aircraft_latlon(max_wait_sec=0.5)
            if msfs_pos is not None:
                print("X-Plane position is still 1,1; using live MSFS position instead")
                return msfs_pos
        print(f"Generating civilian traffic in X-Plane position mode at {float(lat):.5f}, {float(long):.5f}")
        return float(lat), float(long)

    try:
        with open("aircraft_position.json", "r", encoding="utf-8") as f:
            position_data = json.load(f)
        json_lat = float(position_data["latitude"])
        json_lon = float(position_data["longitude"])
        if abs(json_lat - 1.0) < 0.0001 and abs(json_lon - 1.0) < 0.0001:
            msfs_pos = fresh_msfs_aircraft_latlon(max_wait_sec=0.5)
            if msfs_pos is not None:
                print("aircraft_position.json is still 1,1; using live MSFS position instead")
                return msfs_pos
        print(f"Generating civilian traffic from aircraft_position.json at {json_lat:.5f}, {json_lon:.5f}")
        return json_lat, json_lon
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
        return 0.0, 0.0


def make_civilian_surface_contact(name, contact_lat, contact_long, speed, bearing):
    contact = Contact(
        name=name,
        tones=[],
        contact_lat=contact_lat,
        contact_long=contact_long,
        speed=speed,
        depth=0,
        bearing=bearing
    )
    contact.internal_type = "Surface-Ship"
    contact.internal_class = "Civilian"
    contact.classification_type = "Surface-Ship"
    contact.classification_class = "Civilian"
    contact.identity_status = "N"
    contact.operator_classified = True
    contact.detected = True
    contact.broadcasting = True
    gaist_model_title_for_contact(contact)
    apply_surface_ship_acoustic_profile(contact)
    return contact


def aishub_config_value(config_obj, key, default=None):
    ais_config = config_obj.get("aishub", {}) if isinstance(config_obj, dict) else {}
    if isinstance(ais_config, dict) and key in ais_config:
        return ais_config.get(key)
    flat_key = f"aishub_{key}"
    if isinstance(config_obj, dict) and flat_key in config_obj:
        return config_obj.get(flat_key)
    return default


def aishub_enabled(config_obj):
    username = aishub_username(config_obj)
    enabled = aishub_config_value(config_obj, "enabled", bool(username))
    return bool(enabled and username)


def aishub_username(config_obj):
    return str(aishub_config_value(config_obj, "username", os.environ.get("AISHUB_USERNAME", "")) or "").strip()


def aishub_max_contacts(config_obj):
    try:
        value = int(aishub_config_value(config_obj, "max_contacts", os.environ.get("AISHUB_MAX_CONTACTS", AISHUB_DEFAULT_MAX_CONTACTS)))
    except (TypeError, ValueError):
        value = AISHUB_DEFAULT_MAX_CONTACTS
    return max(1, min(1000, value))


def aishub_interval_minutes(config_obj):
    try:
        value = int(aishub_config_value(config_obj, "interval_minutes", os.environ.get("AISHUB_INTERVAL_MINUTES", 180)))
    except (TypeError, ValueError):
        value = 180
    return max(1, value)


def aishub_bounds(config_obj):
    bounds = aishub_config_value(config_obj, "bounds", None)
    if not isinstance(bounds, dict):
        bounds = {}
    result = {}
    for key in ("latmin", "latmax", "lonmin", "lonmax"):
        value = bounds.get(key, aishub_config_value(config_obj, key, None))
        if value is None or value == "":
            continue
        try:
            result[key] = float(value)
        except (TypeError, ValueError):
            continue
    return result


def normalize_ais_float(value, default=0.0):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return float(default)
    if not math.isfinite(result):
        return float(default)
    return result


def aishub_vessel_dimensions(vessel):
    a = max(0.0, normalize_ais_float(vessel.get("A"), 0.0))
    b = max(0.0, normalize_ais_float(vessel.get("B"), 0.0))
    c = max(0.0, normalize_ais_float(vessel.get("C"), 0.0))
    d = max(0.0, normalize_ais_float(vessel.get("D"), 0.0))
    length_m = a + b
    width_m = c + d
    return length_m, width_m, length_m * max(1.0, width_m)


def aishub_is_civilian_vessel(vessel):
    try:
        vessel_type = int(vessel.get("TYPE", 0) or 0)
    except (TypeError, ValueError):
        vessel_type = 0
    if vessel_type == 35:
        return False
    return 30 <= vessel_type <= 89


def aishub_fetch_vessels(config_obj, force=False):
    global aishub_last_fetch_at, aishub_last_error
    if not aishub_enabled(config_obj):
        return []
    now = time.time()
    if not force and now - aishub_last_fetch_at < AISHUB_MIN_REFRESH_SECONDS:
        return []

    params = {
        "username": aishub_username(config_obj),
        "format": 1,
        "output": "json",
        "compress": 0,
        "interval": aishub_interval_minutes(config_obj)
    }
    params.update(aishub_bounds(config_obj))
    url = AISHUB_API_URL + "?" + urllib.parse.urlencode(params)
    try:
        with urllib.request.urlopen(url, timeout=30) as response:
            payload = json.loads(response.read().decode("utf-8"))
    except Exception as exc:
        aishub_last_error = str(exc)
        print(f"[AISHub] fetch failed: {exc}")
        aishub_last_fetch_at = now
        return []

    aishub_last_fetch_at = now
    meta = payload[0] if isinstance(payload, list) and payload else {}
    if isinstance(meta, dict) and meta.get("ERROR"):
        aishub_last_error = str(meta)
        print(f"[AISHub] API error: {aishub_last_error}")
        return []
    vessels = payload[1] if isinstance(payload, list) and len(payload) > 1 and isinstance(payload[1], list) else []
    aishub_last_error = ""
    return vessels


def aishub_civilian_contacts_from_vessels(vessels, max_contacts):
    ranked = []
    seen_mmsi = set()
    for vessel in vessels:
        if not isinstance(vessel, dict) or not aishub_is_civilian_vessel(vessel):
            continue
        mmsi = str(vessel.get("MMSI", "")).strip()
        if not mmsi or mmsi in seen_mmsi:
            continue
        seen_mmsi.add(mmsi)
        lat_value = normalize_ais_float(vessel.get("LATITUDE"), None)
        lon_value = normalize_ais_float(vessel.get("LONGITUDE"), None)
        if lat_value is None or lon_value is None or not (-90 <= lat_value <= 90 and -180 <= lon_value <= 180):
            continue
        length_m, width_m, size_score = aishub_vessel_dimensions(vessel)
        ranked.append((size_score, length_m, width_m, vessel))

    ranked.sort(key=lambda item: (item[0], item[1], item[2]), reverse=True)
    contacts_created = []
    for _, length_m, width_m, vessel in ranked[:max_contacts]:
        name = str(vessel.get("NAME") or vessel.get("MMSI") or "AIS CIV").strip()[:32]
        speed = normalize_ais_float(vessel.get("SOG"), 0.0)
        if speed >= 102.3:
            speed = 0.0
        bearing = normalize_ais_float(vessel.get("HEADING"), 511.0)
        if bearing >= 360.0:
            bearing = normalize_ais_float(vessel.get("COG"), 0.0)
            if bearing >= 360.0:
                bearing = 0.0
        contact = make_civilian_surface_contact(
            name=name,
            contact_lat=normalize_ais_float(vessel.get("LATITUDE"), 0.0),
            contact_long=normalize_ais_float(vessel.get("LONGITUDE"), 0.0),
            speed=max(0.0, speed),
            bearing=bearing % 360.0
        )
        contact.source = "AISHub"
        contact.aishub_mmsi = str(vessel.get("MMSI", ""))
        contact.aishub_imo = str(vessel.get("IMO", ""))
        contact.aishub_type = int(vessel.get("TYPE", 0) or 0)
        contact.length_m = length_m
        contact.width_m = width_m
        contact.track_number = int(vessel.get("MMSI", contact.track_number) or contact.track_number) % 100000
        contacts_created.append(contact)
    return contacts_created


def refresh_aishub_civilian_contacts(config_obj, force=False):
    global aishub_disabled_notice_shown
    if not aishub_enabled(config_obj):
        if not aishub_disabled_notice_shown:
            if not aishub_username(config_obj):
                print("[AISHub] disabled: set aishub.username in config.json or AISHUB_USERNAME")
            else:
                print("[AISHub] disabled by config")
            aishub_disabled_notice_shown = True
        return 0
    aishub_disabled_notice_shown = False
    vessels = aishub_fetch_vessels(config_obj, force=force)
    if not vessels:
        return 0
    contacts[:] = [contact for contact in contacts if getattr(contact, "source", "") != "AISHub"]
    max_contacts = aishub_max_contacts(config_obj)
    ais_contacts = aishub_civilian_contacts_from_vessels(vessels, max_contacts)
    contacts.extend(ais_contacts)
    print(f"[AISHub] imported {len(ais_contacts)} civilian vessels from {len(vessels)} AIS records, largest-first cap {max_contacts}")
    return len(ais_contacts)

def whale_acoustic_profile():
    base_shift = random.uniform(0.85, 1.15)
    source_db = random.uniform(82, 94)
    tones = [
        WhaleCallTone(95 * base_shift, source_db, "Whale Moan", sweep_hz=35, period_sec=random.uniform(6.0, 11.0), phase=random.random(), duty=0.55, harmonics=3, harmonic_drop=11),
        WhaleCallTone(180 * base_shift, source_db - 6, "Whale Harmonic", sweep_hz=75, period_sec=random.uniform(5.0, 9.0), phase=random.random(), duty=0.46, harmonics=3, harmonic_drop=10),
        WhaleCallTone(360 * base_shift, source_db - 12, "Whale Upsweep", sweep_hz=160, period_sec=random.uniform(4.5, 8.5), phase=random.random(), duty=0.32, harmonics=2, harmonic_drop=11),
        WhaleCallTone(720 * base_shift, source_db - 18, "Whale Phrase", sweep_hz=260, period_sec=random.uniform(3.5, 7.5), phase=random.random(), duty=0.26, harmonics=2, harmonic_drop=12),
        WhaleCallTone(random.uniform(1300, 2300), source_db - 24, "Whale Chirp", sweep_hz=random.uniform(450, 850), period_sec=random.uniform(2.8, 5.8), phase=random.random(), duty=0.18, harmonics=1),
        WhaleCallTone(random.uniform(2600, 3400), source_db - 32, "Whale High Call", sweep_hz=random.uniform(250, 700), period_sec=random.uniform(4.0, 7.0), phase=random.random(), duty=0.14, harmonics=1)
    ]
    return tones


def make_whale_contact(name, contact_lat, contact_long, speed, bearing):
    contact = Contact(
        name=name,
        tones=whale_acoustic_profile(),
        contact_lat=contact_lat,
        contact_long=contact_long,
        speed=speed,
        depth=random.uniform(30, 260),
        bearing=bearing
    )
    contact.internal_type = "Biological"
    contact.internal_class = "Whale"
    contact.classification_type = "Biological"
    contact.classification_class = "Whale"
    contact.identity_status = "N"
    contact.operator_classified = True
    contact.detected = False
    contact.broadcasting = False
    contact.acoustic_profile = "Whale"
    return contact

def normalized_range_nm(value, default=0.0):
    if value in (None, False, "", "False", "false"):
        return float(default)
    try:
        return max(0.0, min(250.0, float(value)))
    except (TypeError, ValueError):
        return float(default)


def add_contact_creation_row_for_contact(contact, range_nm=0):
    if contact_define_row_array:
        new_x = contact_define_row_array[-1].row_panel.rect.right + 10
    else:
        new_x = 5

    row = ContactDefineRow(y=0, manager=manager, container=contact_define_container)
    row.row_panel.set_relative_position((new_x, 10))
    contact_define_row_array.append(row)

    row.contact_name_textbox.set_text(str(getattr(contact, "name", "")))
    row.sub_lat_textbox.set_text(f"{float(getattr(contact, 'contact_lat', 0)):.5f}")
    row.sub_long_textbox.set_text(f"{float(getattr(contact, 'contact_long', 0)):.5f}")
    row.sub_range_textbox.set_text(f"{float(range_nm):.1f}")
    row.set_internal_type(
        getattr(contact, "internal_type", "Sub-surface"),
        getattr(contact, "internal_class", "Akula"),
        getattr(contact, "gaist_model_title", "Auto") or "Auto"
    )
    row.set_broadcasting(getattr(contact, "broadcasting", False))
    row.set_team(getattr(contact, "team", "Neutral"))
    row.sub_speed_textbox.set_text(str(int(round(float(getattr(contact, "speed", 0) or 0)))))
    row.sub_bearing_textbox.set_text(str(int(round(float(getattr(contact, "bearing", 0) or 0)))))
    row.sub_depth_textbox.set_text(str(int(round(float(getattr(contact, "depth", 0) or 0)))))
    row.update_from_textboxes()
    update_contact_define_scroll_area()
    return row


def typed_civilian_origin_latlon():
    lat_text = civilian_lat_entry.get_text().strip()
    lon_text = civilian_lon_entry.get_text().strip()
    if lat_text == "" and lon_text == "":
        return None
    try:
        typed_lat = float(lat_text)
        typed_lon = float(lon_text)
    except ValueError:
        print("Civilian traffic skipped: enter valid decimal lat/lon or leave both blank")
        return False
    if not (-90 <= typed_lat <= 90 and -180 <= typed_lon <= 180):
        print("Civilian traffic skipped: lat/lon outside valid range")
        return False
    return typed_lat, typed_lon


def generate_dynamic_civilian_traffic(count=24, origin=None):
    print("Generating civilian traffic around the world")
    created_contacts = []
    start_number = len([c for c in contacts if getattr(c, "internal_class", "") == "Civilian"]) + 1

    skipped = 0
    for index in range(count):
        sea_point = random_global_sea_point()
        if sea_point is None:
            skipped += 1
            continue
        contact_lat, contact_lon, _, spawn_bearing = sea_point
        speed = random.uniform(6, 24)
        course = (spawn_bearing + random.uniform(65, 295)) % 360
        contact = make_civilian_surface_contact(
            f"CIV {start_number + len(created_contacts):02d}",
            contact_lat,
            contact_lon,
            speed,
            course
        )
        contact.source = "RandomCivilian"
        contacts.append(contact)
        created_contacts.append(contact)
        add_contact_creation_row_for_contact(contact, 0)

    print(f"Generated {len(created_contacts)} global civilian surface contacts offshore; skipped {skipped}")
    return created_contacts


def multiplayer_player_traffic_origins():
    origins = []
    own_latlon = None if DEDICATED_HOST_MODE else (current_player_lat_lon_for_mp() if "current_player_lat_lon_for_mp" in globals() else None)
    if own_latlon is not None:
        origins.append({
            "id": MULTIPLAYER_ID,
            "label": MULTIPLAYER_CALLSIGN,
            "lat": float(own_latlon[0]),
            "lon": float(own_latlon[1])
        })
    for peer in multiplayer_peers.values():
        try:
            origins.append({
                "id": str(peer.get("id", "")),
                "label": str(peer.get("callsign", "MP")),
                "lat": float(peer.get("lat")),
                "lon": float(peer.get("lon"))
            })
        except (TypeError, ValueError):
            continue
    return origins


def contact_within_any_origin(contact, origins, radius_nm):
    for origin in origins:
        if haversine(origin["lat"], origin["lon"], contact.contact_lat, contact.contact_long) <= radius_nm:
            return True
    return False


def random_host_civilian_contact(origin, name):
    sea_point = random_sea_point_near(
        origin["lat"],
        origin["lon"],
        min_range_nm=5.0,
        max_range_nm=HOST_CIVILIAN_TRAFFIC_RADIUS_NM,
        min_offshore_nm=CIVILIAN_TRAFFIC_MIN_OFFSHORE_NM,
        attempts=220
    )
    if sea_point is None:
        return None
    contact_lat, contact_lon, _, spawn_bearing = sea_point
    contact = make_civilian_surface_contact(
        name,
        contact_lat,
        contact_lon,
        random.uniform(6, 24),
        (spawn_bearing + random.uniform(65, 295)) % 360
    )
    contact.source = "HostCivilian"
    contact.host_civilian_owner = origin["id"]
    return contact


def update_host_civilian_traffic(force=False):
    global host_civilian_traffic_last_update, multiplayer_last_state_broadcast
    if not multiplayer_is_host_role():
        return 0
    now = time.time()
    if not force and now - host_civilian_traffic_last_update < HOST_CIVILIAN_TRAFFIC_REFRESH_SECONDS:
        return 0
    host_civilian_traffic_last_update = now

    origins = multiplayer_player_traffic_origins()
    if not origins:
        return 0

    before_count = len(contacts)
    contacts[:] = [
        contact for contact in contacts
        if getattr(contact, "source", "") != "HostCivilian" or
        contact_within_any_origin(contact, origins, HOST_CIVILIAN_TRAFFIC_RADIUS_NM * 1.15)
    ]

    created = 0
    start_number = len([contact for contact in contacts if getattr(contact, "internal_class", "") == "Civilian"]) + 1
    for origin in origins:
        nearby = [
            contact for contact in contacts
            if getattr(contact, "source", "") == "HostCivilian" and
            haversine(origin["lat"], origin["lon"], contact.contact_lat, contact.contact_long) <= HOST_CIVILIAN_TRAFFIC_RADIUS_NM
        ]
        needed = max(0, HOST_CIVILIAN_TRAFFIC_TARGET_PER_PLAYER - len(nearby))
        for _ in range(needed):
            contact = random_host_civilian_contact(origin, f"CIV {start_number + created:02d}")
            if contact is None:
                continue
            contacts.append(contact)
            created += 1

    removed = before_count + created - len(contacts)
    if created or removed:
        multiplayer_last_state_broadcast = 0.0
        print(f"[HOST] civilian traffic: +{created}, -{removed}, {len(origins)} player areas within {HOST_CIVILIAN_TRAFFIC_RADIUS_NM:.0f} NM")
    return created


def generate_random_whales(count=None, origin=None):
    if count is None:
        count = random.randint(3, 7)
    if origin is None:
        typed_origin = typed_civilian_origin_latlon()
        if typed_origin is False:
            return []
        origin = typed_origin or current_ownship_latlon()
    if origin is None:
        return []

    origin_lat, origin_lon = origin
    print(f"Generating whale biologics around {origin_lat:.5f}, {origin_lon:.5f}")
    created_contacts = []
    start_number = len([c for c in contacts if getattr(c, "internal_class", "") == "Whale"]) + 1
    group_bearing = random.uniform(0, 360)
    group_range_nm = random.uniform(3, 18)
    group_lat, group_lon = destination_from_bearing(origin_lat, origin_lon, group_bearing, group_range_nm)

    for index in range(count):
        local_range_nm = random.uniform(0.2, 4.5)
        local_bearing = random.uniform(0, 360)
        contact_lat, contact_lon = destination_from_bearing(
            group_lat,
            group_lon,
            local_bearing,
            local_range_nm
        )
        speed = random.uniform(1.5, 6.0)
        course = (local_bearing + random.uniform(80, 280)) % 360
        contact = make_whale_contact(
            f"WHALE {start_number + index:02d}",
            contact_lat,
            contact_lon,
            speed,
            course
        )
        contacts.append(contact)
        created_contacts.append(contact)
        add_contact_creation_row_for_contact(contact, group_range_nm + local_range_nm)

    print(f"Generated {len(created_contacts)} random whale biologic contacts")
    return created_contacts


def ensure_random_whales(min_count=5):
    existing_whales = [
        contact for contact in contacts
        if getattr(contact, "internal_type", "") == "Biological" and
        getattr(contact, "internal_class", "") == "Whale"
    ]
    missing_count = max(0, min_count - len(existing_whales))
    if missing_count <= 0:
        return []

    origin = None
    try:
        if lat is not None and long is not None:
            own_lat = float(lat)
            own_lon = float(long)
            if math.isfinite(own_lat) and math.isfinite(own_lon):
                origin = (own_lat, own_lon)
    except (TypeError, ValueError):
        origin = None

    return generate_random_whales(count=missing_count, origin=origin)


class KiloSubmarine(Contact):
    def __init__(self,name, contact_lat, contact_long, speed, depth, bearing):
        super().__init__("Kilo", [], contact_lat, contact_long, speed, depth, bearing)
        self.prop_noise = 250
        self.pump_noise = 1000
        self.cav_noise = 1500
        self.prop_noise_db = 87.5
        self.pump_noise_db = 77.5
        self.cav_noise_db = 90
        self.source_noise = 100
        self.cav_speed = 6

        # Preload tones
        self.tones = [
            Tone(freq=self.prop_noise, db=self.prop_noise_db, label="Propeller", harmonics=3, harmonic_drop=6),
            Tone(freq=self.pump_noise, db=self.pump_noise_db, label="Pump", harmonics=2, harmonic_drop=4),
            Tone(freq=self.cav_noise, db=self.cav_noise_db, label="Cavitation", harmonics=1)
        ]
        apply_blade_count_metadata(self, "Kilo")


class AkulaSubmarine(Contact):
    def __init__(self,name, contact_lat, contact_long, speed, depth, bearing):
        super().__init__("Akula", [], contact_lat, contact_long, speed, depth, bearing)
        self.prop_noise = 350
        self.pump_noise = 1500
        self.cav_noise = 2500
        self.prop_noise_db = 97.5
        self.pump_noise_db = 87.5
        self.cav_noise_db = 95
        self.source_noise = 115
        self.cav_speed = 9

        self.tones = [
            Tone(freq=self.prop_noise, db=self.prop_noise_db, label="Propeller", harmonics=3, harmonic_drop=6),
            Tone(freq=self.pump_noise, db=self.pump_noise_db, label="Pump", harmonics=2, harmonic_drop=4),
            Tone(freq=self.cav_noise, db=self.cav_noise_db, label="Cavitation", harmonics=1)
        ]
        apply_blade_count_metadata(self, "Akula")


class DeltaIVSubmarine(Contact):
    def __init__(self,name, contact_lat, contact_long, speed, depth, bearing):
        super().__init__("Delta IV", [], contact_lat, contact_long, speed, depth, bearing)
        self.prop_noise = 500
        self.pump_noise = 2000
        self.cav_noise = 2800
        self.prop_noise_db = 107.5
        self.pump_noise_db = 97.5
        self.cav_noise_db = 102.5
        self.source_noise = 120
        self.cav_speed = 10

        self.tones = [
            Tone(freq=self.prop_noise, db=self.prop_noise_db, label="Propeller", harmonics=3, harmonic_drop=6),
            Tone(freq=self.pump_noise, db=self.pump_noise_db, label="Pump", harmonics=2, harmonic_drop=4),
            Tone(freq=self.cav_noise, db=self.cav_noise_db, label="Cavitation", harmonics=1)
        ]
        apply_blade_count_metadata(self, "Delta IV")


class BoreiSubmarine(Contact):
    def __init__(self,name, contact_lat, contact_long, speed, depth, bearing):
        super().__init__("Borei", [], contact_lat, contact_long, speed, depth, bearing)
        self.prop_noise = 300
        self.pump_noise = 1500
        self.cav_noise = 2500
        self.prop_noise_db = 92.5
        self.pump_noise_db = 82.5
        self.cav_noise_db = 90
        self.source_noise = 110
        self.cav_speed = 12

        self.tones = [
            Tone(freq=self.prop_noise, db=self.prop_noise_db, label="Propeller", harmonics=3, harmonic_drop=6),
            Tone(freq=self.pump_noise, db=self.pump_noise_db, label="Pump", harmonics=2, harmonic_drop=4),
            Tone(freq=self.cav_noise, db=self.cav_noise_db, label="Cavitation", harmonics=1)
        ]
        apply_blade_count_metadata(self, "Borei")


class YasenSubmarine(Contact):
    def __init__(self, name, contact_lat, contact_long, speed, depth, bearing):
        super().__init__("Yasen", [], contact_lat, contact_long, speed, depth, bearing)
        self.prop_noise = 500
        self.pump_noise = 1700
        self.cav_noise = 3000
        self.prop_noise_db = 95
        self.pump_noise_db = 82.5
        self.cav_noise_db = 87.5
        self.source_noise = 105
        self.cav_speed = 14

        self.tones = [
            Tone(freq=self.prop_noise, db=self.prop_noise_db, label="Propeller", harmonics=3, harmonic_drop=6),
            Tone(freq=self.pump_noise, db=self.pump_noise_db, label="Pump", harmonics=2, harmonic_drop=4),
            Tone(freq=self.cav_noise, db=self.cav_noise_db, label="Cavitation", harmonics=1)]
        apply_blade_count_metadata(self, "Yasen")
# mathsy variables for acoustic formulas

class BUTECEmitter(Contact):
    def __init__(self, name, contact_lat, contact_long, speed, depth, bearing):
        super().__init__("Yasen", [], contact_lat, contact_long, speed, depth, bearing)
        self.prop_noise = 500
        self.pump_noise = 1000
        self.cav_noise = 1500
        self.prop_noise_db = 100
        self.pump_noise_db = 100
        self.cav_noise_db = 100
        self.source_noise = 100
        self.cav_speed = 0

        self.tones = [
            Tone(freq=self.prop_noise, db=self.prop_noise_db, label="Propeller", harmonics=3, harmonic_drop=6),
            Tone(freq=self.pump_noise, db=self.pump_noise_db, label="Pump", harmonics=2, harmonic_drop=4),
            Tone(freq=self.cav_noise, db=self.cav_noise_db, label="Cavitation", harmonics=1)]
        apply_blade_count_metadata(self, "Emitter")
# mathsy variables for acoustic formulas







sub_classes = {
    'Kilo': KiloSubmarine,
    'Akula': AkulaSubmarine,
    'Delta': DeltaIVSubmarine,
    'Borei': BoreiSubmarine,
    'Yasen': YasenSubmarine,
    'Emitter': BUTECEmitter
}


def resolve_submarine_class_selection(selected_class):
    selected_class = str(selected_class or '').strip()
    if selected_class.lower() == 'random':
        return random.choice(REAL_SUBMARINE_CLASS_OPTIONS)
    return selected_class


# ---------------------------------------------------------------------------
# Acoustic loss model
# ---------------------------------------------------------------------------
# These formulas reduce source dB by spreading and absorption losses. They are
# used by Tone.calc_received_db() when each sonobuoy estimates contact strength.
sub_trail = []
max_trail_length = 100  # Maximum number of points to store

def prop_absorption_loss(distance_nmi, prop_freq_khz):
    """
    Calculate absorption loss for propeller noise.
    Distance in nautical miles, freq in kHz.
    """
    distance_km = distance_nmi * 1.852
    f2 = prop_freq_khz ** 2
    alpha = (0.11 * f2) / (1 + f2) + (44 * f2) / (4100 + f2) + 2.75e-4 * f2 + 0.003  # dB/km
    return alpha * distance_km 


def circular_spreading_loss(distance_nmi):
    distance_m = max(distance_nmi * 1852.0, 1.0)
    return 10 * math.log10(distance_m)


def difar_transmission_loss(distance_nmi, freq_hz):
    return circular_spreading_loss(distance_nmi) + prop_absorption_loss(distance_nmi, freq_hz / 1000)


def difar_background_noise_db(freq_hz):
    freq_hz = max(10.0, float(freq_hz))
    low_frequency_shipping = 56.0 - 9.0 * math.log10(max(freq_hz / 100.0, 1.0))
    wind_sea_noise = 42.0 + 12.0 * math.log10(max(freq_hz / 1000.0, 0.1))
    electronics_floor = 34.0
    powers = [10 ** (level / 10) for level in (low_frequency_shipping, wind_sea_noise, electronics_floor)]
    return 10 * math.log10(sum(powers))


def difar_received_db(source_db, distance_nmi, freq_hz, source_depth_m=0, receiver_depth_m=0):
    return (
        source_db -
        difar_transmission_loss(distance_nmi, freq_hz) -
        environmental_acoustic_loss(distance_nmi, freq_hz, source_depth_m, receiver_depth_m)
    )


def difar_snr_db(source_db, distance_nmi, freq_hz):
    return difar_received_db(source_db, distance_nmi, freq_hz) - difar_background_noise_db(freq_hz)


def calc_difar_bearing_uncert_from_snr(snr_db, range_nm=None):
    snr_db = max(DIFAR_MIN_SNR_DB, float(snr_db))
    snr_margin = snr_db - DIFAR_MIN_SNR_DB

    if range_nm is not None:
        range_nm = max(0.0, float(range_nm))
        if range_nm <= 2.2 and snr_margin >= 3.0:
            return 0.0
        if range_nm <= 2.2:
            return 3.0
        if range_nm <= 3.0 and snr_margin >= 6.0:
            return 0.0
        if range_nm <= 3.0:
            return 6.0

    if snr_margin >= 14.0:
        return 0.0

    return min(35.0, 2.0 + 28.0 * math.exp(-snr_margin / 4.0))


def contact_radial_velocity_towards_receiver_mps(contact, receiver_lat, receiver_lon):
    speed_mps = float(getattr(contact, "speed", 0) or 0) * 0.514444
    if speed_mps == 0:
        return 0.0

    course_rad = math.radians(float(getattr(contact, "bearing", 0) or 0))
    velocity_north = math.cos(course_rad) * speed_mps
    velocity_east = math.sin(course_rad) * speed_mps
    bearing_to_receiver = math.radians(
        haversine_bearing(
            float(contact.contact_lat),
            float(contact.contact_long),
            receiver_lat,
            receiver_lon
        )
    )
    receiver_unit_north = math.cos(bearing_to_receiver)
    receiver_unit_east = math.sin(bearing_to_receiver)
    return velocity_north * receiver_unit_north + velocity_east * receiver_unit_east


def doppler_shifted_frequency(freq_hz, contact, receiver_lat, receiver_lon):
    radial_towards_receiver = contact_radial_velocity_towards_receiver_mps(contact, receiver_lat, receiver_lon)
    radial_towards_receiver = max(-80.0, min(80.0, radial_towards_receiver))
    return freq_hz * SOUND_SPEED_MPS / max(1.0, SOUND_SPEED_MPS - radial_towards_receiver)


ACTIVE_TORPEDO_PING_FREQ_HZ = 3200.0
ACTIVE_TORPEDO_PING_SOURCE_DB = 124.0
ACTIVE_TORPEDO_PING_PERIOD_SEC = 2.0
ACTIVE_TORPEDO_PING_WIDTH_SEC = 0.35


def active_torpedo_ping_phase(torpedo, timestamp):
    return (timestamp - getattr(torpedo, "launch_time", timestamp)) % ACTIVE_TORPEDO_PING_PERIOD_SEC


def active_torpedo_ping_state(torpedo, now=None):
    if now is None:
        now = time.time()
    if (
        getattr(torpedo, "seeker_mode", "PASSIVE") != "ACTIVE" or
        getattr(torpedo, "finished", False) or
        getattr(torpedo, "detonated", False)
    ):
        return False, ACTIVE_TORPEDO_PING_FREQ_HZ, 0.0, 0.0

    phase = active_torpedo_ping_phase(torpedo, now)
    if phase > ACTIVE_TORPEDO_PING_WIDTH_SEC:
        return False, ACTIVE_TORPEDO_PING_FREQ_HZ, 0.0, phase / ACTIVE_TORPEDO_PING_PERIOD_SEC

    pulse_progress = phase / max(0.001, ACTIVE_TORPEDO_PING_WIDTH_SEC)
    sweep = math.sin(pulse_progress * math.pi)
    freq_hz = ACTIVE_TORPEDO_PING_FREQ_HZ + (90.0 * sweep)
    source_db = ACTIVE_TORPEDO_PING_SOURCE_DB - (2.0 * pulse_progress)
    return True, freq_hz, source_db, pulse_progress


def build_passive_difar_emitter_detection(source, source_lat, source_lon, sono_lat, sono_lon,
                                          freq_hz, source_db, label):
    if source_db <= 0:
        return None

    distance_nm = haversine(source_lat, source_lon, sono_lat, sono_lon)
    received_db = difar_received_db(source_db, distance_nm, freq_hz)
    snr_db = received_db - difar_background_noise_db(freq_hz)
    if snr_db < DIFAR_MIN_SNR_DB:
        return None

    bearing = haversine_bearing(sono_lat, sono_lon, source_lat, source_lon)
    uncert = calc_difar_bearing_uncert_from_snr(snr_db, distance_nm)
    if uncert >= 90:
        return None

    return {
        "contact": source,
        "contact_like": False,
        "label": label,
        "bearing": bearing,
        "db": received_db,
        "snr": snr_db,
        "freq": freq_hz,
        "uncert": uncert
    }


def iter_passive_active_difar_detections(sono_lat, sono_lon):
    detections = []
    now = time.time()

    for active_sono in globals().get("active_sono_array", []):
        if not getattr(active_sono, "current_transmitting", False):
            continue
        freq_hz = float(getattr(active_sono, "current_freq_hz", 0.0) or 0.0)
        source_db = float(getattr(active_sono, "current_source_db", 0.0) or 0.0)
        if freq_hz <= 0 or source_db <= 0:
            continue
        source_lat, source_lon = pix_to_latlong(active_sono.x, active_sono.y)
        detection = build_passive_difar_emitter_detection(
            active_sono,
            source_lat,
            source_lon,
            sono_lat,
            sono_lon,
            freq_hz,
            source_db,
            "DICASS PING"
        )
        if detection is not None:
            detections.append(detection)

    for torpedo in globals().get("torp_array", []):
        transmitting, freq_hz, source_db, _ = active_torpedo_ping_state(torpedo, now)
        if transmitting:
            torpedo.last_active_ping_until = now + 0.45
        elif now <= getattr(torpedo, "last_active_ping_until", 0.0):
            transmitting = True
            freq_hz = ACTIVE_TORPEDO_PING_FREQ_HZ
            source_db = ACTIVE_TORPEDO_PING_SOURCE_DB - 6.0
        if not transmitting:
            continue
        torp_lat, torp_lon = pix_to_latlong(torpedo.pos.x, torpedo.pos.y)
        detection = build_passive_difar_emitter_detection(
            torpedo,
            torp_lat,
            torp_lon,
            sono_lat,
            sono_lon,
            freq_hz,
            source_db,
            "ACTIVE TORP"
        )
        if detection is not None:
            detections.append(detection)

    return detections


def pump_absorption_loss(distance_nmi, pump_freq_khz):
    distance_km = distance_nmi * 1.852
    f2 = pump_freq_khz ** 2
    alpha = (0.11 * f2) / (1 + f2) + (44 * f2) / (4100 + f2) + 2.75e-4 * f2 + 0.003
    return alpha * distance_km

def cav_absorption_loss(distance_nmi, cav_freq_khz):
    distance_km = distance_nmi * 1.852
    f2 = cav_freq_khz ** 2
    alpha = (0.11 * f2) / (1 + f2) + (44 * f2) / (4100 + f2) + 2.75e-4 * f2 + 0.003
    return alpha * distance_km



def spherical_spreading_loss(distance_nmi):
    """
    Calculate spherical spreading loss in dB.
    Distance in nautical miles.
    """
    distance_km = distance_nmi * 1.852
    if distance_km <= 0:
        return 0
    # Convert nmi to meters for log10 calculation (optional)
    # or directly calculate log10(distance_nmi) if you want spreading in nmi units
    # Usually spreading is distance-based, so just use nmi directly:
    return 20 * math.log10(distance_km)

noise_amplitude = 0

buffer_duration = 60  # seconds
signal_buffer = np.array([], dtype=np.float32)

chunk_duration = SPECTROGRAM_CHUNK_SECONDS  # seconds per update
samples_per_chunk = int(fs * chunk_duration)

last_signal_update = 0
last_spec_update = 0




def band_limited_noise(min_freq, max_freq, fs, duration, amplitude):
    t = np.linspace(0, duration, int(fs * duration), endpoint=False)
    noise = np.random.normal(0, 1, len(t))
    # Bandpass filter
    b, a = butter(4, [min_freq / (fs / 2), max_freq / (fs / 2)], btype='band')
    filtered = filtfilt(b, a, noise)
    # Normalize and scale
    filtered /= np.max(np.abs(filtered))
    return amplitude * filtered


def get_sonobuoy_by_channel(sono_channel):
    for sono in sono_array:
        if str(sono.channel) == str(sono_channel):
            return sono
    return None


def generate_torpedo_acoustic_signal(t, fs, duration, sono_channel):
    """Generate torpedo runner/impact audio using the existing Tone loss model."""
    sono = get_sonobuoy_by_channel(sono_channel)
    if sono is None:
        return np.zeros_like(t)

    torpedo_signal = np.zeros_like(t)

    for torpedo in torp_array:
        torp_lat, torp_lon = pix_to_latlong(torpedo.pos.x, torpedo.pos.y)

        if torpedo.detonated:
            elapsed = time.time() - torpedo.detonation_started
            if elapsed <= 0.25:
                explosion_tone = Tone(freq=120, db=88, label="Torpedo Explosion")
                received_db = explosion_tone.calc_received_db((sono.x, sono.y), torp_lat, torp_lon)
                burst_amp = min(0.012, 10 ** ((received_db - 102) / 20))
                envelope = np.exp(-28 * t)
                torpedo_signal += band_limited_noise(80, fs / 2 - 100, fs, duration, burst_amp) * envelope
            continue

        if torpedo.finished:
            continue

        blade_freq = 650 + (torpedo.speed_kts * 8)
        runner_tone = Tone(freq=blade_freq, db=108, label="Torpedo Runner", harmonics=2, harmonic_drop=7)
        received_db = runner_tone.calc_received_db((sono.x, sono.y), torp_lat, torp_lon)
        runner_tone.set_received_db_and_freq(sono_channel, received_db, runner_tone.freq)

        for freq, db in runner_tone.get_components(str(sono_channel)):
            amp = 10 ** (db / 20)
            torpedo_signal += amp * np.sin(2 * np.pi * freq * t)

        cav_amp = 10 ** ((received_db - 12) / 20)
        torpedo_signal += band_limited_noise(450, 2600, fs, duration, cav_amp)

        if getattr(torpedo, "seeker_mode", "PASSIVE") == "ACTIVE":
            chunk_start = time.time()
            phases = np.mod(
                chunk_start + t - getattr(torpedo, "launch_time", chunk_start),
                ACTIVE_TORPEDO_PING_PERIOD_SEC
            )
            pulse_mask = phases <= ACTIVE_TORPEDO_PING_WIDTH_SEC
            if np.any(pulse_mask):
                pulse_progress = phases / max(0.001, ACTIVE_TORPEDO_PING_WIDTH_SEC)
                sweep = np.sin(np.clip(pulse_progress, 0.0, 1.0) * math.pi)
                ping_freqs = ACTIVE_TORPEDO_PING_FREQ_HZ + (90.0 * sweep)
                ping_tone = Tone(
                    freq=ACTIVE_TORPEDO_PING_FREQ_HZ,
                    db=ACTIVE_TORPEDO_PING_SOURCE_DB,
                    label="Active Torpedo Ping",
                    harmonics=1
                )
                ping_received_db = ping_tone.calc_received_db((sono.x, sono.y), torp_lat, torp_lon)
                ping_amp = min(0.16, 10 ** ((ping_received_db - 88) / 20))
                pulse_env = np.where(
                    pulse_mask,
                    np.sin(np.clip(pulse_progress, 0.0, 1.0) * math.pi),
                    0.0
                )
                torpedo_signal += ping_amp * pulse_env * np.sin(2 * np.pi * ping_freqs * t)

    return torpedo_signal


def torpedo_explosion_audible(sono_channel, max_distance_nm=4):
    sono = get_sonobuoy_by_channel(sono_channel)
    if sono is None:
        return False

    sono_pos = pygame.Vector2(sono.x, sono.y)
    for torpedo in torp_array:
        if not torpedo.detonated:
            continue
        if time.time() - torpedo.detonation_started > 0.35:
            continue
        if world_distance_nm(torpedo.pos, sono_pos) <= max_distance_nm:
            return True
    return False


def water_impact_signal(t, fs, duration, sono_channel):
    sono = get_sonobuoy_by_channel(sono_channel)
    if sono is None:
        return np.zeros_like(t)

    signal = np.zeros_like(t)
    now = time.time()
    sono_pos = pygame.Vector2(sono.x, sono.y)
    for splash in splash_effects:
        age = now - float(splash.get("start", now))
        if age < 0 or age > 0.55:
            continue
        splash_pos = pygame.Vector2(splash.get("pos", sono_pos))
        distance_nm = max(0.02, world_distance_nm(splash_pos, sono_pos))
        received_db = 96.0 - difar_transmission_loss(distance_nm, 900.0)
        amp = min(0.09, 10 ** ((received_db - 82.0) / 20))
        local_t = age + t
        envelope = np.exp(-16.0 * local_t) * (local_t <= 0.55)
        signal += band_limited_noise(80, min(fs / 2 - 100, 3100), fs, duration, amp) * envelope
    return signal


def water_impact_audible(sono_channel, max_distance_nm=4):
    sono = get_sonobuoy_by_channel(sono_channel)
    if sono is None:
        return False
    sono_pos = pygame.Vector2(sono.x, sono.y)
    now = time.time()
    for splash in splash_effects:
        if now - float(splash.get("start", now)) > 0.65:
            continue
        if world_distance_nm(pygame.Vector2(splash.get("pos", sono_pos)), sono_pos) <= max_distance_nm:
            return True
    return False






def generate_signal_chunk(fs, chunk_duration, contacts,
                          background_db, background_range, sono_channel):
    """Build one synthetic audio/spectrogram chunk for a sonobuoy channel."""

    t = np.linspace(0, chunk_duration, int(fs * chunk_duration), endpoint=False)
    absolute_t = time.time() + t
    signal = np.zeros_like(t)

    for contact in contacts:
        for tone in contact:
            for freq, db in tone.get_components(str(sono_channel)):
                amp = 10 ** (db / 20)
                signal += amp * np.sin(2 * np.pi * freq * absolute_t)

    signal += generate_torpedo_acoustic_signal(t, fs, chunk_duration, sono_channel)
    signal += water_impact_signal(t, fs, chunk_duration, sono_channel)

    background = band_limited_noise(
        background_range[0],
        background_range[1],
        fs,
        chunk_duration,
        10 ** (background_db / 20)
    )

    return (signal + background).astype(np.float32)




def generate_latest_spectrogram_row(signal, fs=7000, nperseg=1000, noverlap=750, nfft=None):
    """Convert the latest signal chunk into a 1-pixel-high pygame surface."""
    nperseg = min(int(nperseg), len(signal))
    noverlap = min(int(noverlap), max(0, nperseg - 1))
    nfft = None if nfft is None else max(int(nfft), nperseg)
    f, t_spec, Sxx = spectrogram(signal, fs=fs, nperseg=nperseg, noverlap=noverlap, nfft=nfft)
    Sxx_db = 10 * np.log10(Sxx + 1e-10)


    db_min = 0
    db_max = 150


   

    # Normalize between 0 and 1
    #Sxx_db_norm = (Sxx_db - Sxx_db.min()) / (Sxx_db.max() - Sxx_db.min())
    Sxx_db_norm = (Sxx_db - db_min) / (db_max - db_min)

    # Take the latest time slice as a horizontal line (last row)
    latest_row = Sxx_db_norm[:, -1]  # this is freq_bins length

    # Apply colormap (viridis, turbo, plasma, inferno)
    color_row = cm.gray(latest_row)[:, :3]  # shape (freq_bins, 3)

    # Convert to 0-255 uint8 and reshape for surface
    row_img = (color_row * 255).astype(np.uint8)

    # We want a 1-pixel high image, width = freq_bins
    # pygame.surfarray.make_surface expects (width, height, 3)
    # So transpose to (width, height, channels)
    row_img = row_img[np.newaxis, :, :]  # shape (1, freq_bins, 3)

    # Swap axes to (width, height, 3)
    row_img = np.swapaxes(row_img, 0, 1)  # shape (freq_bins, 1, 3)
    noise = np.random.normal(0, 5, row_img.shape)
    #noisy_img = np.clip(row_img.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    noisy_img = np.clip(row_img.astype(np.float32), 0, 255).astype(np.uint8)
    # Create pygame surface (width x height)
    surface = pygame.surfarray.make_surface(noisy_img)
    
    return surface


def update_spectrogram_listener():
    global listening_spectrogram_slot

    with listen_audio_lock:
        slot = listening_spectrogram_slot
    if slot is None or in_menu:
        return
    if slot.selected in (None, "None") or not is_numeric_channel(slot.selected):
        with listen_audio_lock:
            listening_spectrogram_slot = None
        for display_slot in spectrogram_slot_array:
            display_slot.sync_listen_button_style()
        return

    if slot.get_passive_sonobuoy_for_slot() is None:
        with listen_audio_lock:
            listening_spectrogram_slot = None
        for display_slot in spectrogram_slot_array:
            display_slot.sync_listen_button_style()
        return


def normalize_listen_audio(audio):
    audio = np.asarray(audio, dtype=np.float32)
    if not audio.size:
        return audio
    audio -= float(np.mean(audio))
    peak = float(np.max(np.abs(audio)))
    if peak > 1e-6:
        audio = audio / peak
    return (np.tanh(audio * 2.0) * 0.20 * master_sound_level).astype(np.float32)


def listen_audio_worker_loop():
    global listening_spectrogram_slot

    chunk_seconds = LISTEN_AUDIO_CHUNK_SECONDS
    idle_sleep = 0.02
    while running_audio:
        with listen_audio_lock:
            slot = listening_spectrogram_slot

        if slot is None or in_menu:
            time.sleep(idle_sleep)
            continue

        try:
            if slot.selected in (None, "None") or not is_numeric_channel(slot.selected):
                with listen_audio_lock:
                    if listening_spectrogram_slot is slot:
                        listening_spectrogram_slot = None
                time.sleep(idle_sleep)
                continue

            if slot.get_passive_sonobuoy_for_slot() is None:
                with listen_audio_lock:
                    if listening_spectrogram_slot is slot:
                        listening_spectrogram_slot = None
                time.sleep(idle_sleep)
                continue

            audio = generate_signal_chunk(
                fs,
                chunk_seconds,
                contacts,
                58,
                (1, min(3499, fs / 2 - 1)),
                int(slot.selected)
            )
            audio = normalize_listen_audio(audio)
            stream.write(audio.tobytes(), exception_on_underflow=False)
        except Exception as exc:
            print(f"[AUDIO] DIFAR listen failed: {exc}")
            with listen_audio_lock:
                if listening_spectrogram_slot is slot:
                    listening_spectrogram_slot = None
            time.sleep(0.2)


def start_listen_audio_thread():
    global listen_audio_thread
    if listen_audio_thread is not None and listen_audio_thread.is_alive():
        return
    listen_audio_thread = Thread(target=listen_audio_worker_loop, daemon=True)
    listen_audio_thread.start()


if not DEDICATED_HOST_MODE:
    start_listen_audio_thread()


def stop_listen_audio():
    global running_audio, listening_spectrogram_slot
    running_audio = False
    with listen_audio_lock:
        listening_spectrogram_slot = None
    try:
        stream.stop_stream()
        stream.close()
        p.terminate()
    except Exception:
        pass


''''''''''''''''''''''

def check_thermocline_side():
    if sub_depth > thermocline_depth and sono.depth < thermocline_depth:


    if sub_depth < thermocline_depth and sono.depth > thermocline_depth:



'''''''''''''''

data_Surface.fill((5,5,10))













default_environment_xbt = XBT(
    temp_seabed=seabed_temp,
    temp_surface=surface_temp,
    thermocline_depth=thermocline_depth,
    max_depth=seabed_depth,
    label="ENV"
)
default_environment_xbt.update()


def draw_combined_xbt_profile(surface, xbt_profile, rect):
    if not xbt_profile or not xbt_profile.profile or not xbt_profile.sound_profile:
        return

    rect = pygame.Rect(rect)
    panel_bg = (5, 5, 10)
    border = (92, 96, 98)
    grid = (34, 44, 48)
    temp_colour = (235, 90, 70)
    speed_colour = (0, 220, 235)
    text_colour = (205, 210, 210)
    muted = (120, 128, 130)
    title_font = scaled_sys_font(11, bold=True)
    tick_font = scaled_sys_font(9)

    plot_rect = rect.inflate(-58, -40)
    plot_rect.left += 34
    plot_rect.top += 20
    plot_rect.width -= 26

    temps_local = [value for _, value in xbt_profile.profile]
    speeds_local = [value for _, value in xbt_profile.sound_profile]
    max_depth_local = max(1.0, max(depth for depth, _ in xbt_profile.sound_profile))

    temp_min = min(temps_local)
    temp_max = max(temps_local)
    temp_span = max(1.0, temp_max - temp_min)
    temp_min -= temp_span * 0.12
    temp_max += temp_span * 0.12
    temp_span = max(1.0, temp_max - temp_min)

    speed_min = min(speeds_local)
    speed_max = max(speeds_local)
    speed_span = max(1.0, speed_max - speed_min)
    speed_min -= speed_span * 0.04
    speed_max += speed_span * 0.04
    speed_span = max(1.0, speed_max - speed_min)

    pygame.draw.rect(surface, panel_bg, rect)
    pygame.draw.rect(surface, border, rect, 1)
    pygame.draw.rect(surface, (0, 0, 0), plot_rect)
    pygame.draw.rect(surface, border, plot_rect, 1)

    surface.blit(title_font.render("TEMP + SOUND SPEED", False, text_colour), (rect.x + 8, rect.y + 4))
    surface.blit(tick_font.render("DEPTH m", False, text_colour), (rect.left + 4, rect.y + 22))
    surface.blit(tick_font.render("TEMP C", False, temp_colour), (plot_rect.left, rect.bottom - 13))
    speed_label = tick_font.render("C m/s", False, speed_colour)
    surface.blit(speed_label, (plot_rect.right - speed_label.get_width(), rect.bottom - 13))

    for i in range(5):
        y = plot_rect.top + i / 4 * plot_rect.height
        pygame.draw.line(surface, grid, (plot_rect.left, y), (plot_rect.right, y), 1)
        depth_tick = i / 4 * max_depth_local
        depth_label = tick_font.render(f"{int(depth_tick)}", False, muted)
        surface.blit(depth_label, (rect.left + 4, y - depth_label.get_height() // 2))

    for i in range(4):
        x = plot_rect.left + i / 3 * plot_rect.width
        pygame.draw.line(surface, grid, (x, plot_rect.top), (x, plot_rect.bottom), 1)
        temp_tick = temp_min + i / 3 * temp_span
        speed_tick = speed_min + i / 3 * speed_span
        temp_label = tick_font.render(f"{temp_tick:.0f}", False, temp_colour)
        speed_label_tick = tick_font.render(f"{speed_tick:.0f}", False, speed_colour)
        surface.blit(temp_label, (x - temp_label.get_width() // 2, plot_rect.bottom + 2))
        surface.blit(speed_label_tick, (x - speed_label_tick.get_width() // 2, plot_rect.top - 11))

    layer_depth = acoustic_layer_depth()
    if 0 <= layer_depth <= max_depth_local:
        layer_y = plot_rect.top + (layer_depth / max_depth_local) * plot_rect.height
        draw_dotted_line(surface, (180, 180, 90), (plot_rect.left, layer_y), (plot_rect.right, layer_y), 1, dash_length=4, gap_length=4)
        layer_label = tick_font.render("LAYER", False, (190, 190, 100))
        surface.blit(layer_label, (plot_rect.left + 3, layer_y + 2))

    axis_depth = deep_sound_channel_axis_depth()
    if 0 <= axis_depth <= max_depth_local:
        axis_y = plot_rect.top + (axis_depth / max_depth_local) * plot_rect.height
        draw_dotted_line(surface, (90, 180, 235), (plot_rect.left, axis_y), (plot_rect.right, axis_y), 1, dash_length=4, gap_length=4)
        axis_label = tick_font.render("DSC", False, (90, 180, 235))
        surface.blit(axis_label, (plot_rect.left + 3, axis_y + 12))

    def temp_point(depth, value):
        return (
            int(plot_rect.left + ((value - temp_min) / temp_span) * plot_rect.width),
            int(plot_rect.top + (depth / max_depth_local) * plot_rect.height)
        )

    def speed_point(depth, value):
        return (
            int(plot_rect.left + ((value - speed_min) / speed_span) * plot_rect.width),
            int(plot_rect.top + (depth / max_depth_local) * plot_rect.height)
        )

    temp_points = [temp_point(depth, value) for depth, value in xbt_profile.profile]
    speed_points = [speed_point(depth, value) for depth, value in xbt_profile.sound_profile]

    if len(temp_points) > 1:
        pygame.draw.aalines(surface, temp_colour, False, temp_points)
        pygame.draw.lines(surface, temp_colour, False, temp_points, 1)
    if len(speed_points) > 1:
        pygame.draw.aalines(surface, speed_colour, False, speed_points)
        pygame.draw.lines(surface, speed_colour, False, speed_points, 1)

    for point in temp_points:
        pygame.draw.circle(surface, temp_colour, point, 1)
    for point in speed_points:
        pygame.draw.circle(surface, speed_colour, point, 1)


def latest_xbt_label():
    if not xbt_profiles:
        return None
    if latest_xbt_profile is not None:
        return latest_xbt_profile.label
    return next(reversed(xbt_profiles.keys()))



def buoy_depth_m_for_ray(buoy):
    depth_value = getattr(buoy, "depth", None)
    try:
        depth_ft = float(depth_value)
    except (TypeError, ValueError):
        depth_ft = 90.0
    return max(0.0, depth_ft * 0.3048)


def world_distance_nm(start_world, end_world):
    start_lat, start_lon = pix_to_latlong(start_world.x, start_world.y)
    end_lat, end_lon = pix_to_latlong(end_world.x, end_world.y)
    return haversine(start_lat, start_lon, end_lat, end_lon)


def find_raytrace_buoy_near_world(world_pos, radius_px=48):
    if world_pos is None:
        return None
    world_pos = pygame.Vector2(world_pos)
    pointer_screen = world_to_right_display_pos(world_pos)
    best_buoy = None
    best_screen_distance = float(radius_px)
    best_world_distance_nm = 0.75

    for buoy in list(sono_array) + list(active_sono_array):
        buoy_world = pygame.Vector2(getattr(buoy, "x", 0), getattr(buoy, "y", 0))
        buoy_screen = world_to_right_display_pos(buoy_world)
        if pointer_screen is not None and buoy_screen is not None:
            screen_distance = pygame.Vector2(pointer_screen).distance_to(buoy_screen)
            if screen_distance <= best_screen_distance:
                best_screen_distance = screen_distance
                best_buoy = buoy
                continue

        world_distance = world_distance_nm(world_pos, buoy_world)
        if world_distance <= best_world_distance_nm:
            best_world_distance_nm = world_distance
            best_buoy = buoy

    return best_buoy


def sound_speed_at_depth_for_profile(profile, depth_m):
    if profile is None or not getattr(profile, "sound_profile", None):
        return sound_speed_at_depth(depth_m)
    sound_profile = profile.sound_profile
    depth_m = max(0.0, min(float(depth_m), sound_profile[-1][0]))
    for index in range(1, len(sound_profile)):
        d0, c0 = sound_profile[index - 1]
        d1, c1 = sound_profile[index]
        if d0 <= depth_m <= d1:
            frac = (depth_m - d0) / max(1.0, d1 - d0)
            return c0 + (c1 - c0) * frac
    return sound_profile[-1][1]


def trace_acoustic_ray(source_depth_m, angle_deg, range_nm, step_nm=0.04, profile=None):
    points = []
    max_depth = max(50.0, float(seabed_depth or 1000.0))
    source_depth_m = max(0.0, min(float(source_depth_m or 0.0), max_depth))
    range_nm = max(0.1, float(range_nm or 0.1))
    c0 = max(1.0, sound_speed_at_depth_for_profile(profile, source_depth_m))
    launch_angle = math.radians(abs(float(angle_deg)))
    horizontal_slowness = math.cos(launch_angle) / c0
    vertical_direction = -1.0 if angle_deg < 0 else 1.0
    if abs(angle_deg) < 0.01:
        vertical_direction = 1.0
    depth_m = source_depth_m
    range_done = 0.0
    points.append((0.0, depth_m))
    steps = max(10, min(700, int(math.ceil(range_nm / max(0.02, step_nm)))))
    step_nm = range_nm / steps
    step_m = step_nm * 1852.0
    for _ in range(steps):
        c_here = max(1.0, sound_speed_at_depth_for_profile(profile, depth_m))
        c_for_bend = max(1.0, c0 + (c_here - c0) * 7.0)
        cos_angle = horizontal_slowness * c_for_bend
        if cos_angle >= 0.999:
            cos_angle = 0.999
            vertical_direction *= -1.0
        elif cos_angle <= 0.02:
            cos_angle = 0.02
        ray_angle = math.acos(cos_angle)
        depth_m += vertical_direction * math.tan(ray_angle) * step_m
        if depth_m < 0.0:
            depth_m = -depth_m
            vertical_direction = 1.0
        elif depth_m > max_depth:
            depth_m = max_depth - (depth_m - max_depth)
            depth_m = max(0.0, depth_m)
            vertical_direction = -1.0
        range_done += step_nm
        points.append((min(range_done, range_nm), max(0.0, min(depth_m, max_depth))))
    return points


def build_ray_trace_result_from_xbt(xbt_profile, target_world):
    source_pos = getattr(xbt_profile, "position", None)
    if xbt_profile is None or source_pos is None:
        return None
    source_world = pygame.Vector2(source_pos)
    target_world = pygame.Vector2(target_world)
    range_nm = max(0.1, world_distance_nm(source_world, target_world))
    source_lat, source_lon = pix_to_latlong(source_world.x, source_world.y)
    target_lat, target_lon = pix_to_latlong(target_world.x, target_world.y)
    bearing_deg = haversine_bearing(source_lat, source_lon, target_lat, target_lon)
    source_depth_m = 0.0
    angles = [-32, -24, -16, -9, -4, 0, 4, 9, 16, 24, 32]
    paths = [trace_acoustic_ray(source_depth_m, angle, range_nm, profile=xbt_profile) for angle in angles]
    return {"source": source_world, "target": target_world, "range_nm": range_nm, "bearing": bearing_deg, "source_depth_m": source_depth_m, "paths": paths, "angles": angles, "axis_depth": deep_sound_channel_axis_depth(), "layer_depth": acoustic_layer_depth(), "label": getattr(xbt_profile, "label", "XBT")}


def begin_ray_trace_selection():
    global ray_trace_mode, ray_trace_source_buoy, ray_trace_status_text, xbt_panel_visible, xbt_panel_selected_label
    xbt_panel_visible = True
    if xbt_panel_selected_label is None:
        xbt_panel_selected_label = latest_xbt_label()
    selected_profile = selected_xbt_for_panel()
    if selected_profile is None or getattr(selected_profile, "position", None) is None:
        ray_trace_mode = "OFF"
        ray_trace_status_text = "Select/deploy an XBT first."
    else:
        ray_trace_mode = "SELECT_POINT"
        ray_trace_status_text = f"{selected_profile.label}: click raytrace endpoint."
    ray_trace_source_buoy = None
    update_xbt_panel_selector()
    xbt_panel_close_button.show()
    xbt_raytrace_button.show()
    xbt_raytrace_clear_button.show()
    if xbt_panel_select_dropdown is not None:
        xbt_panel_select_dropdown.show()


def clear_ray_trace():
    global ray_trace_mode, ray_trace_source_buoy, ray_trace_result, ray_trace_status_text
    ray_trace_mode = "OFF"
    ray_trace_source_buoy = None
    ray_trace_result = None
    ray_trace_status_text = "Ray trace cleared."


def handle_ray_trace_click(world_pos):
    global ray_trace_mode, ray_trace_source_buoy, ray_trace_result, ray_trace_status_text
    if ray_trace_mode == "SELECT_POINT":
        selected_profile = selected_xbt_for_panel()
        ray_trace_result = build_ray_trace_result_from_xbt(selected_profile, world_pos)
        if ray_trace_result is None:
            ray_trace_status_text = "Select/deploy an XBT first."
            ray_trace_mode = "OFF"
            return True
        ray_trace_status_text = f"{ray_trace_result['label']} {ray_trace_result['range_nm']:.1f} NM {ray_trace_result['bearing']:03.0f} deg."
        ray_trace_mode = "OFF"
        return True
    return False


def draw_ray_trace_graph(surface, result, rect):
    if not result:
        return
    rect = pygame.Rect(rect)
    plot_rect = rect.inflate(-54, -34)
    plot_rect.left += 34
    plot_rect.top += 16
    plot_rect.width -= 14
    title_font = scaled_sys_font(11, bold=True)
    tick_font = scaled_sys_font(9)
    border = (92, 96, 98)
    grid = (34, 44, 48)
    text_colour = (205, 210, 210)
    pygame.draw.rect(surface, (5, 5, 10), rect)
    pygame.draw.rect(surface, border, rect, 1)
    pygame.draw.rect(surface, (0, 0, 0), plot_rect)
    pygame.draw.rect(surface, border, plot_rect, 1)
    surface.blit(title_font.render("RAY TRACE", False, text_colour), (rect.x + 8, rect.y + 4))
    range_nm = max(0.1, result.get("range_nm", 0.1))
    max_depth = max(50.0, float(seabed_depth or 1000.0))
    for i in range(5):
        y = plot_rect.top + i / 4 * plot_rect.height
        pygame.draw.line(surface, grid, (plot_rect.left, y), (plot_rect.right, y), 1)
        depth_label = tick_font.render(f"{int(i / 4 * max_depth)}", False, text_colour)
        surface.blit(depth_label, (rect.left + 4, y - depth_label.get_height() // 2))
    for i in range(4):
        x = plot_rect.left + i / 3 * plot_rect.width
        pygame.draw.line(surface, grid, (x, plot_rect.top), (x, plot_rect.bottom), 1)
        range_label = tick_font.render(f"{i / 3 * range_nm:.1f}", False, text_colour)
        surface.blit(range_label, (x - range_label.get_width() // 2, plot_rect.bottom + 2))
    for depth, colour, label in ((result.get("layer_depth", 0), (185, 180, 80), "LAYER"), (result.get("axis_depth", 0), (90, 180, 235), "DSC")):
        if 0 <= depth <= max_depth:
            y = plot_rect.top + depth / max_depth * plot_rect.height
            draw_dotted_line(surface, colour, (plot_rect.left, y), (plot_rect.right, y), 1, dash_length=5, gap_length=4)
            surface.blit(tick_font.render(label, False, colour), (plot_rect.left + 3, y + 2))
    ray_colours = [(60, 185, 255), (80, 225, 170), (230, 210, 80), (235, 120, 80)]
    for index, path in enumerate(result.get("paths", [])):
        points = []
        for range_point, depth_point in path:
            x = plot_rect.left + (range_point / range_nm) * plot_rect.width
            y = plot_rect.top + (depth_point / max_depth) * plot_rect.height
            points.append((x, y))
        if len(points) >= 2:
            pygame.draw.aalines(surface, ray_colours[index % len(ray_colours)], False, points)
    source_y = plot_rect.top + (result.get("source_depth_m", 0.0) / max_depth) * plot_rect.height
    pygame.draw.circle(surface, (0, 240, 120), (plot_rect.left, int(source_y)), 4)
    row = f"{result.get('label', 'BUOY')}  {range_nm:.1f} NM  {result.get('bearing', 0):03.0f} deg"
    surface.blit(tick_font.render(row, False, text_colour), (plot_rect.left + 6, rect.bottom - 12))


def draw_ray_trace_overlay(surface):
    if not ray_trace_result:
        return
    start_screen = world_to_right_display_pos(ray_trace_result["source"])
    end_screen = world_to_right_display_pos(ray_trace_result["target"])
    if start_screen is None or end_screen is None:
        return
    draw_dotted_line(surface, (115, 170, 205), start_screen, end_screen, 2, dash_length=8, gap_length=6)
    pygame.draw.circle(surface, (115, 220, 205), start_screen, 5, 1)
    pygame.draw.circle(surface, (115, 220, 205), end_screen, 5, 1)


def update_xbt_panel_selector():
    global xbt_panel_select_dropdown, xbt_panel_selected_label

    options = list(xbt_profiles.keys()) or ["None"]
    selected = xbt_panel_selected_label if xbt_panel_selected_label in options else options[-1]
    if selected == "None":
        xbt_panel_selected_label = None
    else:
        xbt_panel_selected_label = selected

    was_visible = bool(xbt_panel_visible)
    if xbt_panel_select_dropdown is not None:
        xbt_panel_select_dropdown.kill()
    xbt_panel_select_dropdown = pygame_gui.elements.UIDropDownMenu(
        options_list=options,
        starting_option=selected,
        relative_rect=pygame.Rect((690, 548), (160, 26)),
        manager=manager,
        expansion_height_limit=180
    )
    register_ui_element(xbt_panel_select_dropdown, xbt_panel_select_dropdown.relative_rect)
    if was_visible:
        xbt_panel_select_dropdown.show()
    else:
        xbt_panel_select_dropdown.hide()


def selected_xbt_for_panel():
    label = xbt_panel_selected_label if xbt_panel_selected_label in xbt_profiles else latest_xbt_label()
    return xbt_profiles.get(label) if label is not None else None


def draw_xbt_panel(surface):
    panel_rect = pygame.Rect((8, 542), (930, 430))
    if not xbt_panel_visible:
        surface.fill((5, 5, 10), rect=panel_rect)
        return

    pygame.draw.rect(surface, (5, 5, 10), panel_rect)
    pygame.draw.rect(surface, (92, 96, 98), panel_rect, 1)
    title_font = scaled_sys_font(15, bold=True)
    small_font = scaled_sys_font(12)

    profile = selected_xbt_for_panel()
    if profile is None:
        surface.blit(title_font.render("XBT PROFILE", False, (0, 220, 235)), (panel_rect.x + 88, panel_rect.y + 10))
        surface.blit(small_font.render("No XBT deployed.", False, (220, 220, 220)), (panel_rect.x + 88, panel_rect.y + 42))
        return

    surface.blit(title_font.render(f"{profile.label} XBT PROFILE", False, (0, 220, 235)), (panel_rect.x + 278, panel_rect.y + 10))
    if ray_trace_result is not None:
        graph_rect = pygame.Rect(panel_rect.x + 18, panel_rect.y + 44, 420, panel_rect.height - 92)
        ray_rect = pygame.Rect(panel_rect.x + 456, panel_rect.y + 44, 456, panel_rect.height - 92)
        draw_combined_xbt_profile(surface, profile, graph_rect)
        draw_ray_trace_graph(surface, ray_trace_result, ray_rect)
    else:
        graph_rect = pygame.Rect(panel_rect.x + 18, panel_rect.y + 44, panel_rect.width - 36, panel_rect.height - 92)
        draw_combined_xbt_profile(surface, profile, graph_rect)

    layer = acoustic_layer_depth()
    axis = deep_sound_channel_axis_depth()
    row = (
        f"Layer {layer:.0f} m   DSC {axis:.0f} m   Seabed {seabed_depth:.0f} m   "
        f"C0 {sound_speed_at_depth(0):.1f}   "
        f"CL {sound_speed_at_depth(layer):.1f}   "
        f"CB {sound_speed_at_depth(seabed_depth):.1f} m/s"
    )
    surface.blit(small_font.render(row, False, (220, 220, 220)), (panel_rect.x + 18, panel_rect.bottom - 34))
    surface.blit(small_font.render(ray_trace_status_text, False, (130, 210, 230)), (panel_rect.x + 18, panel_rect.bottom - 18))





ray_angle_array = [340]  # angles in degrees


mouse_down = False
measure_dragging = False
measure_drag_start_world = None
measure_drag_current_world = None
measure_locked = None
measure_last_click_time = 0.0
measure_last_click_pos = None


def internal_pos_on_right_display(internal_pos):
    return internal_pos.x >= INTERNAL_WIDTH / 2 and 0 <= internal_pos.y <= INTERNAL_HEIGHT


def map_screen_pos_to_world(map_pos):
    return pygame.Vector2(
        ((map_pos.x - map_surf.get_width() / 2) / map_layer.zoom) + map_centre_x,
        ((map_pos.y - map_surf.get_height() / 2) / map_layer.zoom) + map_centre_y
    )


def radar_screen_pos_to_world(map_pos):
    radar_width, radar_height = map_surf.get_size()
    center = pygame.Vector2(radar_width / 2, radar_height / 2)
    radar_radius = min(radar_width, radar_height) * 0.43
    radar_range_nm = radar_range_options[radar_range_index]
    pixels_per_nm = radar_radius / radar_range_nm
    rotation_deg = hdg if radar_orientation == "TRACK" else 0

    relative = pygame.Vector2(map_pos) - center
    distance_px = relative.length()
    if distance_px > radar_radius:
        return None

    display_bearing = math.degrees(math.atan2(relative.x, -relative.y)) % 360
    true_bearing = (display_bearing + rotation_deg) % 360
    distance_nm = distance_px / pixels_per_nm
    start_lat, start_lon = pix_to_latlong(*latlong_to_pix(player_pos.x, player_pos.y))
    target_lat, target_lon = destination_from_bearing(start_lat, start_lon, true_bearing, distance_nm)
    return pygame.Vector2(latlong_to_pix(target_lat, target_lon))


def right_display_pos_to_world(internal_pos):
    if not internal_pos_on_right_display(internal_pos):
        return None
    map_pos = pygame.Vector2(internal_pos.x - INTERNAL_WIDTH / 2, internal_pos.y)
    if display_mode == "RADAR":
        return radar_screen_pos_to_world(map_pos)
    return map_screen_pos_to_world(map_pos)


def measurement_snap_candidates():
    candidates = []
    for contact in contacts:
        if is_dicass_ping_contact(contact) or not contact_is_radar_visible(contact):
            continue
        if hasattr(contact, "contact_lat") and hasattr(contact, "contact_long"):
            candidates.append(pygame.Vector2(latlong_to_pix(contact.contact_lat, contact.contact_long)))

    for sono in sono_array:
        candidates.append(pygame.Vector2(sono.x, sono.y))

    for active_sono in active_sono_array:
        candidates.append(pygame.Vector2(active_sono.x, active_sono.y))

    for xbt_item in xbt_array:
        if hasattr(xbt_item, "x") and hasattr(xbt_item, "y"):
            candidates.append(pygame.Vector2(xbt_item.x, xbt_item.y))
        elif isinstance(xbt_item, (tuple, list)) and len(xbt_item) >= 2:
            candidates.append(pygame.Vector2(xbt_item[0], xbt_item[1]))

    for torpedo in torp_array:
        candidates.append(pygame.Vector2(torpedo.pos))

    return candidates


def snap_measurement_world(world_pos, radius_px=16):
    if world_pos is None:
        return None

    world_pos = pygame.Vector2(world_pos)
    pointer_screen = world_to_right_display_pos(world_pos)
    if pointer_screen is None:
        return world_pos

    best_world = world_pos
    best_distance = radius_px
    for candidate in measurement_snap_candidates():
        candidate_screen = world_to_right_display_pos(candidate)
        if candidate_screen is None:
            continue
        distance = pygame.Vector2(pointer_screen).distance_to(candidate_screen)
        if distance <= best_distance:
            best_distance = distance
            best_world = candidate

    return pygame.Vector2(best_world)


def snapped_right_display_pos_to_world(internal_pos):
    return snap_measurement_world(right_display_pos_to_world(internal_pos))


def world_to_right_display_pos(world_pos):
    world_pos = pygame.Vector2(world_pos)
    if display_mode == "RADAR":
        radar_width, radar_height = map_surf.get_size()
        center = pygame.Vector2(radar_width / 2, radar_height / 2)
        radar_radius = min(radar_width, radar_height) * 0.43
        radar_range_nm = radar_range_options[radar_range_index]
        pixels_per_nm = radar_radius / radar_range_nm
        rotation_deg = hdg if radar_orientation == "TRACK" else 0
        player_world = pygame.Vector2(latlong_to_pix(player_pos.x, player_pos.y))
        screen_pos = radar_point_from_world(world_pos, player_world, center, pixels_per_nm, rotation_deg)
        if center.distance_to(screen_pos) > radar_radius:
            return None
        return screen_pos
    return pygame.Vector2(map_layer.translate_point(world_pos))


def measure_text(start_world, end_world):
    start_lat, start_lon = pix_to_latlong(start_world.x, start_world.y)
    end_lat, end_lon = pix_to_latlong(end_world.x, end_world.y)
    distance_nm = haversine(start_lat, start_lon, end_lat, end_lon)
    bearing_deg = haversine_bearing(start_lat, start_lon, end_lat, end_lon)
    return f"{distance_nm:.2f} NM  {bearing_deg:03.0f} deg"


def draw_measurement(surface, start_world, end_world, locked=False):
    start_screen = world_to_right_display_pos(start_world)
    end_screen = world_to_right_display_pos(end_world)
    if start_screen is None or end_screen is None:
        return

    colour = (175, 180, 185) if locked else (135, 140, 145)
    draw_dotted_line(surface, colour, start_screen, end_screen, 2, dash_length=7, gap_length=5)
    pygame.draw.circle(surface, colour, start_screen, 5, 1)
    pygame.draw.circle(surface, colour, end_screen, 5, 1)

    midpoint = (pygame.Vector2(start_screen) + pygame.Vector2(end_screen)) * 0.5
    label = font.render(measure_text(pygame.Vector2(start_world), pygame.Vector2(end_world)), False, colour)
    label_bg = pygame.Surface((label.get_width() + 8, label.get_height() + 6), pygame.SRCALPHA)
    label_bg.fill((0, 0, 0, 185))
    label_bg.blit(label, (4, 3))
    label_x = max(4, min(int(midpoint.x + 8), surface.get_width() - label_bg.get_width() - 4))
    label_y = max(4, min(int(midpoint.y - label_bg.get_height() - 8), surface.get_height() - label_bg.get_height() - 4))
    surface.blit(label_bg, (label_x, label_y))


def draw_measurements(surface):
    if measure_locked is not None:
        draw_measurement(surface, measure_locked["start"], measure_locked["end"], locked=True)
    if measure_dragging and measure_drag_start_world is not None and measure_drag_current_world is not None:
        draw_measurement(surface, measure_drag_start_world, measure_drag_current_world, locked=False)


def search_pattern_float(entry, default, minimum=None, maximum=None):
    try:
        value = float(entry.get_text())
    except ValueError:
        return default
    if minimum is not None:
        value = max(minimum, value)
    if maximum is not None:
        value = min(maximum, value)
    return value


def search_pattern_int(entry, default, minimum=None, maximum=None):
    return int(round(search_pattern_float(entry, default, minimum, maximum)))


def latlon_to_navigraph(lat_deg, lon_deg):
    def split_dms(value, deg_width):
        total_seconds = int(round(abs(float(value)) * 3600))
        degrees = total_seconds // 3600
        minutes = (total_seconds % 3600) // 60
        seconds = total_seconds % 60
        return f"{degrees:0{deg_width}d}{minutes:02d}{seconds:02d}"

    lat_hemi = "N" if lat_deg >= 0 else "S"
    lon_hemi = "E" if lon_deg >= 0 else "W"
    return f"{split_dms(lat_deg, 2)}{lat_hemi}{split_dms(lon_deg, 3)}{lon_hemi}"


def navigraph_to_latlon(token):
    match = re.fullmatch(r"\s*(\d{2})(\d{2})(\d{2})([NS])\s*(\d{3})(\d{2})(\d{2})([EW])\s*", token, re.IGNORECASE)
    if not match:
        return None

    lat_deg, lat_min, lat_sec, lat_hemi, lon_deg, lon_min, lon_sec, lon_hemi = match.groups()
    lat_value = int(lat_deg) + int(lat_min) / 60 + int(lat_sec) / 3600
    lon_value = int(lon_deg) + int(lon_min) / 60 + int(lon_sec) / 3600
    if lat_hemi.upper() == "S":
        lat_value *= -1
    if lon_hemi.upper() == "W":
        lon_value *= -1
    if not (-90 <= lat_value <= 90 and -180 <= lon_value <= 180):
        return None
    return lat_value, lon_value


def parse_navigraph_waypoints(text):
    waypoints = []
    pattern = re.compile(r"\d{2}\d{2}\d{2}[NS]\s*\d{3}\d{2}\d{2}[EW]", re.IGNORECASE)
    for match in pattern.finditer(text or ""):
        latlon = navigraph_to_latlon(match.group(0))
        if latlon is not None:
            waypoints.append(pygame.Vector2(latlong_to_pix(latlon[0], latlon[1])))
    return waypoints


def set_contact_latlon(contact, lat_value, lon_value):
    contact.contact_lat = float(lat_value)
    contact.contact_long = float(lon_value)
    x, y = latlong_to_pix(contact.contact_lat, contact.contact_long)
    contact.pos = pygame.Vector2(x, y)
    if hasattr(contact, "contact_rect"):
        contact.contact_rect.center = (round(contact.pos.x), round(contact.pos.y))


def shadow_target_name_from_saved_config(data):
    if not isinstance(data, dict):
        return ""
    return str(data.get("shadow_target", data.get("shadow_target_name", "")) or "").strip()


def shadow_distance_from_saved_config(data):
    if not isinstance(data, dict):
        return 5.0
    try:
        return max(0.1, float(data.get("shadow_distance_nm", data.get("shadow_distance", 5.0)) or 5.0))
    except (TypeError, ValueError):
        return 5.0


def contact_shadow_target(contact):
    target_name = str(getattr(contact, "shadow_target_name", "") or "").strip()
    if not target_name:
        return None
    contact_name = str(getattr(contact, "name", "") or "").strip().lower()
    target_key = target_name.lower()
    for candidate in contacts:
        if candidate is contact or is_dicass_ping_contact(candidate):
            continue
        if str(getattr(candidate, "name", "") or "").strip().lower() == target_key:
            return candidate
    return None


def shadow_slot_for_contact(contact, target):
    distance_nm = max(0.1, float(getattr(contact, "shadow_distance_nm", 5.0) or 5.0))
    target_bearing = float(getattr(target, "bearing", 0.0) or 0.0) % 360.0
    return destination_from_bearing(
        float(getattr(target, "contact_lat", 0.0) or 0.0),
        float(getattr(target, "contact_long", 0.0) or 0.0),
        (target_bearing + 180.0) % 360.0,
        distance_nm
    )


def command_contact_shadow_following(contact):
    target = contact_shadow_target(contact)
    if target is None:
        return False
    slot_lat, slot_lon = shadow_slot_for_contact(contact, target)
    range_to_slot_nm = haversine(contact.contact_lat, contact.contact_long, slot_lat, slot_lon)
    target_speed = max(0.0, float(getattr(target, "speed", getattr(contact, "speed", 0.0)) or 0.0))
    max_shadow_speed = max(
        target_speed + shadow_catchup_speed_margin_kts,
        float(getattr(contact, "original_speed", target_speed) or target_speed),
        float(getattr(contact, "ship_underway_speed", target_speed) or target_speed),
        float(getattr(contact, "commanded_speed", target_speed) or target_speed)
    )
    if range_to_slot_nm <= 0.05:
        commanded_heading = float(getattr(target, "bearing", getattr(contact, "bearing", 0.0)) or 0.0) % 360.0
        commanded_speed = target_speed
    else:
        commanded_heading = haversine_bearing(contact.contact_lat, contact.contact_long, slot_lat, slot_lon)
        commanded_speed = min(max_shadow_speed, target_speed + min(shadow_catchup_speed_margin_kts, range_to_slot_nm * 2.0))
    contact.commanded_heading = commanded_heading
    contact.commanded_speed = max(0.0, commanded_speed)
    contact.desired_bearing = contact.commanded_heading
    contact.target_speed = contact.commanded_speed
    contact.shadow_status = f"Shadow {getattr(target, 'name', '?')} {range_to_slot_nm:.1f}NM HDG {contact.commanded_heading:03.0f} SPD {contact.commanded_speed:.1f}"
    return True


def apply_shadow_command_dynamics(contact, dt_seconds):
    ensure_ship_command_state(contact)
    dt_seconds = max(0.0, float(dt_seconds or 0.0))
    current_speed = max(0.0, float(getattr(contact, "speed", 0.0) or 0.0))
    target_speed = max(0.0, float(getattr(contact, "commanded_speed", current_speed) or 0.0))
    speed_rate = ship_command_accel_kts_s if target_speed > current_speed else ship_command_decel_kts_s
    contact.speed = ship_approach_value(current_speed, target_speed, speed_rate * dt_seconds)
    contact.bearing = ship_approach_bearing(
        float(getattr(contact, "bearing", 0.0) or 0.0),
        float(getattr(contact, "commanded_heading", getattr(contact, "bearing", 0.0)) or 0.0),
        ship_command_turn_rate_dps * dt_seconds
    )


def update_contact_shadow_following(contact, dt_seconds):
    if not command_contact_shadow_following(contact):
        return False
    apply_shadow_command_dynamics(contact, dt_seconds)
    if getattr(contact, "speed", 0) != 0:
        contact.move(dt_seconds)
    return True


def snap_contact_shadow_to_slot(contact):
    target = contact_shadow_target(contact)
    if target is None:
        return False
    shadow_lat, shadow_lon = shadow_slot_for_contact(contact, target)
    set_contact_latlon(contact, shadow_lat, shadow_lon)
    contact.bearing = float(getattr(target, "bearing", getattr(contact, "bearing", 0.0)) or 0.0) % 360.0
    contact.speed = float(getattr(target, "speed", getattr(contact, "speed", 0.0)) or 0.0)
    contact.commanded_heading = contact.bearing
    contact.commanded_speed = contact.speed
    return True


def update_all_contact_shadow_following(passes=3, snap=False):
    updated = False
    for _ in range(max(1, int(passes))):
        pass_updated = False
        for contact in list(contacts):
            if is_dicass_ping_contact(contact):
                continue
            did_update = snap_contact_shadow_to_slot(contact) if snap else command_contact_shadow_following(contact)
            if did_update:
                pass_updated = True
        updated = updated or pass_updated
        if not pass_updated:
            break
    return updated


def parse_route_timestamp(value, default=None):
    if value is None or value == "":
        return default
    if isinstance(value, (int, float)):
        return float(value)
    text = str(value).strip()
    if not text:
        return default
    try:
        return float(text)
    except ValueError:
        pass
    if text.endswith("Z"):
        text = text[:-1] + "+00:00"
    try:
        return datetime.datetime.fromisoformat(text).timestamp()
    except ValueError:
        return default


def route_timestamp_to_config(value):
    if value is None:
        return None
    try:
        return datetime.datetime.fromtimestamp(float(value), datetime.timezone.utc).isoformat().replace("+00:00", "Z")
    except (TypeError, ValueError, OSError):
        return None


def route_waypoints_to_config(points):
    return [[float(lat_value), float(lon_value)] for lat_value, lon_value in points]


def parse_route_waypoints(value):
    points = []
    if value is None:
        return points
    if isinstance(value, str):
        for point in parse_navigraph_waypoints(value):
            points.append(pix_to_latlong(point.x, point.y))
        if points:
            return points
        decimal_pattern = re.compile(r"(-?\d+(?:\.\d+)?)\s*[, ]\s*(-?\d+(?:\.\d+)?)")
        for match in decimal_pattern.finditer(value):
            lat_value = float(match.group(1))
            lon_value = float(match.group(2))
            if -90 <= lat_value <= 90 and -180 <= lon_value <= 180:
                points.append((lat_value, lon_value))
        return points
    if isinstance(value, dict):
        value = value.get("waypoints", [])
    if not isinstance(value, list):
        return points
    for item in value:
        try:
            if isinstance(item, dict):
                lat_value = float(item.get("lat", item.get("latitude")))
                lon_value = float(item.get("lon", item.get("longitude")))
            elif isinstance(item, str):
                nested = parse_route_waypoints(item)
                points.extend(nested)
                continue
            else:
                lat_value = float(item[0])
                lon_value = float(item[1])
            if -90 <= lat_value <= 90 and -180 <= lon_value <= 180:
                points.append((lat_value, lon_value))
        except (TypeError, ValueError, IndexError):
            continue
    return points


def route_total_distance_nm(points, loop=False):
    if len(points) < 2:
        return 0.0
    total = 0.0
    leg_count = len(points) if loop else len(points) - 1
    for index in range(leg_count):
        start = points[index]
        end = points[(index + 1) % len(points)]
        total += haversine(start[0], start[1], end[0], end[1])
    return total


def route_position_at_elapsed(points, speed_kts, elapsed_seconds, loop=False):
    if not points:
        return None
    if len(points) == 1 or speed_kts <= 0:
        lat_value, lon_value = points[0]
        return lat_value, lon_value, 0.0, 0, False

    remaining_nm = max(0.0, float(speed_kts) * max(0.0, float(elapsed_seconds)) / 3600.0)
    total_nm = route_total_distance_nm(points, loop)
    if loop and total_nm > 1e-6:
        remaining_nm = remaining_nm % total_nm

    index = 0
    while index < len(points) - 1 or loop:
        start = points[index]
        next_index = (index + 1) % len(points)
        end = points[next_index]
        leg_nm = haversine(start[0], start[1], end[0], end[1])
        if leg_nm <= 1e-6:
            index = next_index
            if not loop and index >= len(points) - 1:
                break
            continue
        bearing = haversine_bearing(start[0], start[1], end[0], end[1])
        if remaining_nm <= leg_nm:
            lat_value, lon_value = destination_from_bearing(start[0], start[1], bearing, remaining_nm)
            return lat_value, lon_value, bearing, index, True
        remaining_nm -= leg_nm
        index = next_index
        if not loop and index >= len(points) - 1:
            break

    last = points[-1]
    previous = points[-2] if len(points) > 1 else last
    bearing = haversine_bearing(previous[0], previous[1], last[0], last[1]) if previous != last else 0.0
    return last[0], last[1], bearing, max(0, len(points) - 1), False


def route_points_with_contact_start(contact, points):
    points = list(points or [])
    if not points:
        return points, False
    try:
        current = (float(contact.contact_lat), float(contact.contact_long))
    except (TypeError, ValueError):
        return points, False
    if not (-90 <= current[0] <= 90 and -180 <= current[1] <= 180):
        return points, False
    if haversine(current[0], current[1], points[0][0], points[0][1]) <= ship_route_arrival_radius_nm:
        return points, False
    return [current] + points, True


def route_points_for_config(contact):
    points = list(getattr(contact, "route_waypoints", []) or [])
    if getattr(contact, "route_runtime_start_inserted", False) and len(points) > 1:
        return points[1:]
    return points


def configure_ship_route(contact, route_config, now=None):
    points = parse_route_waypoints(route_config.get("waypoints", route_config) if isinstance(route_config, dict) else route_config)
    points, inserted_start = route_points_with_contact_start(contact, points)
    contact.route_waypoints = points
    contact.route_runtime_start_inserted = inserted_start
    contact.route_index = 0
    contact.route_active = False
    contact.route_status = "No route"
    if not points:
        return False

    route_source = route_config if isinstance(route_config, dict) else {}
    speed_default = float(getattr(contact, "commanded_speed", getattr(contact, "speed", 0.0)) or getattr(contact, "speed", 0.0) or 0.0)
    contact.route_speed_kts = max(0.0, float(route_source.get("speed", route_source.get("speed_kts", speed_default)) or 0.0))
    contact.route_loop = bool(route_source.get("loop", route_source.get("route_loop", False)))
    start_default = now if now is not None else time.time()
    if inserted_start:
        contact.route_started_at_utc = start_default
    else:
        contact.route_started_at_utc = parse_route_timestamp(
            route_source.get("started_at_utc", route_source.get("start_utc", route_source.get("started_at"))),
            start_default
        )
    contact.route_active = len(points) > 1 and contact.route_speed_kts > 0.0
    contact.route_manual_paused = False
    contact.route_status = f"Route: {len(points)} WPT" if contact.route_active else "Route idle"
    return True


def apply_ship_route_elapsed_position(contact, now=None):
    if getattr(contact, "internal_type", "") != "Surface-Ship" or not getattr(contact, "route_waypoints", None):
        return False
    if now is None:
        now = time.time()
    speed_kts = float(getattr(contact, "route_speed_kts", None) or getattr(contact, "speed", 0.0) or 0.0)
    started_at = parse_route_timestamp(getattr(contact, "route_started_at_utc", None), now)
    result = route_position_at_elapsed(
        getattr(contact, "route_waypoints", []),
        speed_kts,
        now - started_at,
        bool(getattr(contact, "route_loop", False))
    )
    if result is None:
        return False
    lat_value, lon_value, bearing, route_index, active = result
    set_contact_latlon(contact, lat_value, lon_value)
    contact.bearing = bearing % 360.0
    contact.speed = speed_kts if active else 0.0
    contact.commanded_heading = contact.bearing
    contact.commanded_speed = contact.speed
    contact.route_index = route_index
    contact.route_active = bool(active)
    contact.route_manual_paused = False
    contact.route_status = f"Route WPT {route_index + 1}/{len(contact.route_waypoints)}" if active else "Route complete"
    return True


def update_ship_route_following(contact):
    if getattr(contact, "internal_type", "") != "Surface-Ship" or not getattr(contact, "route_active", False):
        return False
    points = getattr(contact, "route_waypoints", [])
    if len(points) < 2:
        contact.route_active = False
        contact.route_status = "No route"
        return False
    index = min(max(0, int(getattr(contact, "route_index", 0) or 0)), len(points) - 1)
    next_index = (index + 1) % len(points)
    if not getattr(contact, "route_loop", False) and index >= len(points) - 1:
        contact.route_active = False
        contact.commanded_speed = 0.0
        contact.route_status = "Route complete"
        return False
    target_lat, target_lon = points[next_index]
    distance_nm = haversine(contact.contact_lat, contact.contact_long, target_lat, target_lon)
    if distance_nm <= ship_route_arrival_radius_nm:
        contact.route_index = next_index
        if not getattr(contact, "route_loop", False) and next_index >= len(points) - 1:
            contact.route_active = False
            contact.commanded_speed = 0.0
            contact.route_status = "Route complete"
            return False
        next_index = (contact.route_index + 1) % len(points)
        target_lat, target_lon = points[next_index]
    contact.commanded_heading = haversine_bearing(contact.contact_lat, contact.contact_long, target_lat, target_lon)
    contact.commanded_speed = max(0.0, float(getattr(contact, "route_speed_kts", getattr(contact, "speed", 0.0)) or 0.0))
    contact.route_status = f"Route WPT {contact.route_index + 1}/{len(points)}"
    return True



def ship_route_config_from_saved_config(data):
    if not isinstance(data, dict):
        return None
    route_config = data.get("route", data.get("ship_route"))
    if route_config is not None:
        return route_config
    route_text = str(data.get("route_text", "") or "").strip()
    if route_text:
        waypoints = parse_route_waypoints(route_text)
        if waypoints:
            return {
                "waypoints": waypoints,
                "speed_kts": data.get("route_speed_kts", data.get("route_speed", data.get("speed"))),
                "loop": data.get("route_loop", False),
                "started_at_utc": data.get("route_started_at_utc", data.get("route_start_utc", data.get("route_started_at", time.time())))
            }
    waypoints = data.get("route_waypoints", data.get("waypoints"))
    if waypoints is None:
        return None
    return {
        "waypoints": waypoints,
        "speed_kts": data.get("route_speed_kts", data.get("route_speed", data.get("speed"))),
        "loop": data.get("route_loop", False),
        "started_at_utc": data.get("route_started_at_utc", data.get("route_start_utc", data.get("route_started_at")))
    }


def route_text_from_saved_config(data):
    if not isinstance(data, dict):
        return ""
    route_text = str(data.get("route_text", "") or "").strip()
    if route_text:
        return route_text
    route_config = data.get("route", data.get("ship_route"))
    if isinstance(route_config, dict):
        waypoints = route_config.get("waypoints", [])
    else:
        waypoints = data.get("route_waypoints", data.get("waypoints", []))
    formatted = []
    for point in waypoints or []:
        try:
            lat_value = float(point.get("lat", point.get("latitude"))) if isinstance(point, dict) else float(point[0])
            lon_value = float(point.get("lon", point.get("longitude"))) if isinstance(point, dict) else float(point[1])
        except (TypeError, ValueError, IndexError, AttributeError):
            continue
        formatted.append(f"{lat_value:.5f},{lon_value:.5f}")
    return " ".join(formatted)
def ship_route_config_from_contact(contact):
    points = route_points_for_config(contact)
    if not points:
        return None
    return {
        "waypoints": route_waypoints_to_config(points),
        "speed_kts": float(getattr(contact, "route_speed_kts", getattr(contact, "speed", 0.0)) or 0.0),
        "loop": bool(getattr(contact, "route_loop", False)),
        "started_at_utc": route_timestamp_to_config(getattr(contact, "route_started_at_utc", None))
    }

def merge_live_contact_state_into_saved(saved_contact, contact):
    if contact is None:
        return saved_contact
    saved_contact["classification_type"] = str(getattr(contact, "classification_type", "Unknown"))
    saved_contact["classification_class"] = str(getattr(contact, "classification_class", "Unknown"))
    saved_contact["identity_status"] = str(getattr(contact, "identity_status", "P"))
    saved_contact["country"] = str(getattr(contact, "country", "Unknown"))
    saved_contact["operator_classified"] = bool(getattr(contact, "operator_classified", False))
    saved_contact["detected"] = bool(getattr(contact, "detected", False))
    saved_contact["commanded_speed"] = float(getattr(contact, "commanded_speed", getattr(contact, "speed", 0.0)) or 0.0)
    saved_contact["commanded_heading"] = float(getattr(contact, "commanded_heading", getattr(contact, "bearing", 0.0)) or 0.0) % 360.0
    saved_contact["ship_underway_speed"] = float(getattr(contact, "ship_underway_speed", saved_contact["commanded_speed"]) or 0.0)
    if getattr(contact, "route_speed_kts", None) is not None:
        saved_contact["route_speed_kts"] = float(getattr(contact, "route_speed_kts", 0.0) or 0.0)
    saved_contact["route_active"] = bool(getattr(contact, "route_active", False))
    saved_contact["route_manual_paused"] = bool(getattr(contact, "route_manual_paused", False))
    saved_contact["route_index"] = int(getattr(contact, "route_index", 0) or 0)
    saved_contact["route_status"] = str(getattr(contact, "route_status", "No route"))
    shadow_target = str(getattr(contact, "shadow_target_name", "") or "").strip()
    if shadow_target:
        saved_contact["shadow_target"] = shadow_target
        saved_contact["shadow_distance_nm"] = float(getattr(contact, "shadow_distance_nm", 5.0) or 5.0)
    return saved_contact


def apply_saved_contact_state(contact, data):
    if not isinstance(data, dict):
        return contact
    contact.classification_type = str(data.get("classification_type", getattr(contact, "classification_type", "Unknown")))
    contact.classification_class = str(data.get("classification_class", getattr(contact, "classification_class", "Unknown")))
    contact.identity_status = str(data.get("identity_status", getattr(contact, "identity_status", "P")))
    contact.country = str(data.get("country", getattr(contact, "country", "Unknown")))
    contact.operator_classified = bool(data.get("operator_classified", getattr(contact, "operator_classified", False)))
    contact.detected = bool(data.get("detected", getattr(contact, "detected", False)))
    contact.broadcasting = bool(data.get("broadcasting", getattr(contact, "broadcasting", False)))
    contact.team = str(data.get("team", getattr(contact, "team", "Neutral")))
    contact.shadow_target_name = shadow_target_name_from_saved_config(data)
    contact.shadow_distance_nm = shadow_distance_from_saved_config(data)
    if "commanded_speed" in data:
        contact.commanded_speed = max(0.0, float(data.get("commanded_speed", 0.0) or 0.0))
    if "commanded_heading" in data:
        contact.commanded_heading = float(data.get("commanded_heading", getattr(contact, "bearing", 0.0)) or 0.0) % 360.0
    if "ship_underway_speed" in data:
        contact.ship_underway_speed = max(0.0, float(data.get("ship_underway_speed", 0.0) or 0.0))
    if "route_speed_kts" in data and getattr(contact, "route_waypoints", None):
        contact.route_speed_kts = max(0.0, float(data.get("route_speed_kts", 0.0) or 0.0))
    if getattr(contact, "route_waypoints", None):
        if "route_index" in data:
            contact.route_index = max(0, min(int(data.get("route_index", 0) or 0), len(contact.route_waypoints) - 1))
        if "route_active" in data:
            contact.route_active = bool(data.get("route_active", getattr(contact, "route_active", False)))
        contact.route_manual_paused = bool(data.get("route_manual_paused", getattr(contact, "route_manual_paused", False)))
        contact.route_status = str(data.get("route_status", getattr(contact, "route_status", "No route")))
    return contact


def current_aircraft_latlon_for_search():
    return float(player_pos.x), float(player_pos.y)


def player_is_surface_ship():
    return dropdown_value(MULTIPLAYER_PLAYER_TYPE) == "Ship"


def player_is_submarine():
    return dropdown_value(MULTIPLAYER_PLAYER_TYPE) == "Submarine"


def player_is_ship_or_sub():
    return player_is_surface_ship() or player_is_submarine()


def autosave_ship_route_to_config(contact):
    route_config = ship_route_config_from_contact(contact)
    if route_config is None:
        return False
    cfg_path = globals().get("config_path", "config.json")
    try:
        with open(cfg_path, "r") as f:
            saved_config = json.load(f)
    except (OSError, json.JSONDecodeError) as exc:
        print(f"Ship route autosave failed: could not read config.json: {exc}")
        return False

    entries = saved_config.get("submarines", [])
    contact_name = str(getattr(contact, "name", ""))
    matched = None
    for entry in entries:
        if str(entry.get("name", "")) == contact_name:
            matched = entry
            break
    if matched is None:
        print(f"Ship route autosave skipped: no config entry named {contact_name}")
        return False

    matched["route"] = route_config
    try:
        with open(cfg_path, "w") as f:
            json.dump(saved_config, f, indent=4)
    except OSError as exc:
        print(f"Ship route autosave failed: could not write config.json: {exc}")
        return False
    print(f"Ship route autosaved for {contact_name}")
    return True

def live_contact_to_config_entry(contact):
    entry = {
        "name": str(getattr(contact, "name", "Contact")),
        "latitude": float(getattr(contact, "contact_lat", 0.0) or 0.0),
        "longitude": float(getattr(contact, "contact_long", 0.0) or 0.0),
        "range": 0.0,
        "speed": float(getattr(contact, "speed", 0.0) or 0.0),
        "depth": float(getattr(contact, "depth", 0.0) or 0.0),
        "bearing": float(getattr(contact, "bearing", 0.0) or 0.0) % 360.0,
        "class": str(getattr(contact, "internal_class", "Akula") or "Akula"),
        "internal_type": str(getattr(contact, "internal_type", "Unknown") or "Unknown"),
        "internal_class": str(getattr(contact, "internal_class", "Unknown") or "Unknown"),
        "broadcasting": bool(getattr(contact, "broadcasting", False)),
        "team": str(getattr(contact, "team", "Neutral") or "Neutral"),
        "model_library": normalize_model_library(getattr(contact, "model_library", MODEL_LIBRARY_AUTO)),
        "model": str(getattr(contact, "gaist_model_title", "Auto") or "Auto")
    }
    if entry["internal_type"] != "Sub-surface":
        entry["class"] = "Akula"
    route_config = ship_route_config_from_contact(contact)
    if route_config is not None:
        entry["route"] = route_config
    merge_live_contact_state_into_saved(entry, contact)
    return entry


def save_all_live_contacts_to_config(reason="autosave"):
    cfg_path = globals().get("config_path", "config.json")
    try:
        with open(cfg_path, "r") as f:
            saved_config = json.load(f)
    except (OSError, json.JSONDecodeError) as exc:
        print(f"[SERVER] contact {reason} failed: could not read config.json: {exc}")
        return False
    entries = saved_config.setdefault("submarines", [])
    live_names = {
        str(getattr(contact, "name", "") or "").strip()
        for contact in contacts
        if str(getattr(contact, "name", "") or "").strip() and not is_dicass_ping_contact(contact)
    }
    entries[:] = [
        entry
        for entry in entries
        if not isinstance(entry, dict) or not str(entry.get("name", "") or "").strip() or str(entry.get("name", "") or "").strip() in live_names
    ]
    by_name = {str(entry.get("name", "")): entry for entry in entries if isinstance(entry, dict)}
    for contact in contacts:
        name = str(getattr(contact, "name", "") or "").strip()
        if not name or is_dicass_ping_contact(contact):
            continue
        live_entry = live_contact_to_config_entry(contact)
        if name in by_name:
            by_name[name].update(live_entry)
        else:
            entries.append(live_entry)
            by_name[name] = live_entry
    try:
        with open(cfg_path, "w") as f:
            json.dump(saved_config, f, indent=4)
    except OSError as exc:
        print(f"[SERVER] contact {reason} failed: could not write config.json: {exc}")
        return False
    print(f"[SERVER] contact {reason} saved {len(entries)} config contacts")
    return True


def maybe_server_contact_autosave():
    global server_contact_autosave_last
    if not DEDICATED_HOST_MODE:
        return
    now = time.time()
    if now - server_contact_autosave_last >= SERVER_CONTACT_AUTOSAVE_SECONDS:
        server_contact_autosave_last = now
        save_all_live_contacts_to_config("autosave")


def server_console_reader():
    while True:
        try:
            line = sys.stdin.readline()
        except Exception:
            break
        if not line:
            break
        server_command_queue.put(line.strip())


def start_server_console_thread():
    global server_command_thread
    if server_command_thread is not None:
        return
    server_command_thread = Thread(target=server_console_reader, daemon=True)
    server_command_thread.start()


def parse_server_contact_name(name):
    name = str(name or "").strip().strip("\"\"")
    return contact_by_name(name)


def handle_server_command(line):
    if not line:
        return
    lower = line.lower()
    if lower in ("help", "?"):
        print("[SERVER] commands: hdg <contact> <000>, spd <contact> <kts>, stop <contact>, delete <contact>, <contact> shadow <target>, <contact> unshadow, save")
        return
    if lower == "save":
        save_all_live_contacts_to_config("manual save")
        return
    if " shadow " in lower:
        split_at = lower.index(" shadow ")
        source_name = line[:split_at].strip()
        target_name = line[split_at + len(" shadow "):].strip()
        contact = parse_server_contact_name(source_name)
        target = parse_server_contact_name(target_name)
        if contact is None or target is None:
            print("[SERVER] shadow failed: contact or target not found")
            return
        contact.shadow_target_name = getattr(target, "name", target_name)
        contact.shadow_distance_nm = 5.0
        update_all_contact_shadow_following()
        print(f"[SERVER] {contact.name} shadowing {target.name}")
        return
    if lower.endswith(" unshadow"):
        source_name = line[:-len(" unshadow")].strip()
        contact = parse_server_contact_name(source_name)
        if contact is None:
            print("[SERVER] unshadow failed: contact not found")
            return
        contact.shadow_target_name = ""
        contact.shadow_status = "No shadow target"
        print(f"[SERVER] {contact.name} shadow cleared")
        return
    parts = line.split()
    if len(parts) >= 3 and parts[0].lower() in ("hdg", "heading"):
        contact = parse_server_contact_name(" ".join(parts[1:-1]))
        if contact is None:
            print("[SERVER] heading failed: contact not found")
            return
        request_ship_heading(contact, float(parts[-1]))
        print(f"[SERVER] {contact.name} heading {float(parts[-1]) % 360.0:03.0f}")
        return
    if len(parts) >= 3 and parts[0].lower() in ("spd", "speed"):
        contact = parse_server_contact_name(" ".join(parts[1:-1]))
        if contact is None:
            print("[SERVER] speed failed: contact not found")
            return
        request_ship_speed(contact, parts[-1])
        print(f"[SERVER] {contact.name} speed {parts[-1]}")
        return
    if len(parts) >= 2 and parts[0].lower() in ("delete", "del", "rm"):
        contact = parse_server_contact_name(" ".join(parts[1:]))
        if contact is None:
            print("[SERVER] delete failed: contact not found")
            return
        contact_name = str(getattr(contact, "name", "Contact"))
        delete_contact(contact)
        save_all_live_contacts_to_config("delete")
        print(f"[SERVER] deleted {contact_name}")
        return
    if len(parts) >= 2 and parts[0].lower() == "stop":
        contact = parse_server_contact_name(" ".join(parts[1:]))
        if contact is None:
            print("[SERVER] stop failed: contact not found")
            return
        request_ship_stop(contact)
        print(f"[SERVER] {contact.name} stopped")
        return
    print(f"[SERVER] unknown command: {line}")


def process_server_commands():
    while not server_command_queue.empty():
        handle_server_command(server_command_queue.get())

def assign_ship_route_from_text(contact, route_text, loop=False):
    if contact is None or getattr(contact, "internal_type", "") != "Surface-Ship":
        print("Ship route ignored: select a surface ship contact first")
        return False
    imported = parse_route_waypoints(route_text)
    if not imported:
        print(f"Ship route ignored for track {getattr(contact, 'track_number', '?')}: no valid waypoints")
        return False
    current_point = (float(contact.contact_lat), float(contact.contact_long))
    if haversine(current_point[0], current_point[1], imported[0][0], imported[0][1]) > ship_route_arrival_radius_nm:
        imported = [current_point] + imported
    speed_kts = max(
        ship_default_underway_speed_kts,
        float(getattr(contact, "commanded_speed", 0.0) or 0.0),
        float(getattr(contact, "speed", 0.0) or 0.0),
        float(getattr(contact, "route_speed_kts", 0.0) or 0.0)
    )
    route_config = {
        "waypoints": imported,
        "speed_kts": speed_kts,
        "loop": loop,
        "started_at_utc": time.time()
    }
    if configure_ship_route(contact, route_config):
        target_index = min(1, len(imported) - 1)
        contact.commanded_speed = speed_kts
        contact.commanded_heading = haversine_bearing(
            contact.contact_lat,
            contact.contact_long,
            imported[target_index][0],
            imported[target_index][1]
        )
        print(f"Ship track {getattr(contact, 'track_number', '?')} assigned route with {len(imported)} waypoints at {speed_kts:.1f} kt")
        autosave_ship_route_to_config(contact)
        return True
    return False

def clamp_float(value, default, minimum=None, maximum=None):
    try:
        result = float(value)
    except (TypeError, ValueError):
        result = float(default)
    if minimum is not None:
        result = max(float(minimum), result)
    if maximum is not None:
        result = min(float(maximum), result)
    return result


def update_nav_control_visibility():
    visible = (display_mode == "NAV")
    for element in nav_elements:
        if visible:
            element.show()
        else:
            element.hide()
    if player_is_submarine():
        nav_depth_label.show() if visible else nav_depth_label.hide()
        nav_depth_entry.show() if visible else nav_depth_entry.hide()
    else:
        nav_depth_label.hide()
        nav_depth_entry.hide()


def apply_nav_command_entries(format_fields=True):
    global ownship_commanded_heading, ownship_commanded_speed, ownship_commanded_depth
    ownship_commanded_heading = clamp_float(nav_heading_entry.get_text(), ownship_commanded_heading, 0.0, 359.9) % 360
    ownship_commanded_speed = clamp_float(nav_speed_entry.get_text(), ownship_commanded_speed, 0.0, 45.0 if player_is_surface_ship() else 35.0)
    ownship_commanded_depth = clamp_float(nav_depth_entry.get_text(), ownship_commanded_depth, 0.0, 2000.0)
    if not player_is_submarine():
        ownship_commanded_depth = 0.0
    if format_fields:
        nav_heading_entry.set_text(f"{ownship_commanded_heading:03.0f}")
        nav_speed_entry.set_text(f"{ownship_commanded_speed:.1f}")
        nav_depth_entry.set_text(f"{ownship_commanded_depth:.0f}")


def import_ownship_route_from_nav_entry():
    global ownship_route_waypoints, ownship_route_index, ownship_route_active, ownship_route_status
    imported = parse_navigraph_waypoints(nav_route_entry.get_text())
    ownship_route_waypoints = [pygame.Vector2(point) for point in imported]
    ownship_route_index = 0
    ownship_route_active = bool(ownship_route_waypoints)
    ownship_route_status = f"Route: {len(ownship_route_waypoints)} WPT" if ownship_route_active else "No valid route"
    nav_route_status_label.set_text(ownship_route_status)


def update_ship_sub_ownship(dt_seconds):
    global lat, long, hdg, alt, ownship_current_heading, ownship_last_heading, ownship_heading_rate_dps
    global ownship_current_speed, ownship_current_depth, ownship_commanded_heading, ownship_commanded_speed, ownship_commanded_depth, ownship_route_index, ownship_route_active, ownship_route_status
    global ownship_control_initialized_track
    contact = selected_ownship_control_contact()
    if contact is None:
        ownship_route_status = "No controllable contact"
        nav_route_status_label.set_text(ownship_route_status)
        return

    lat = float(contact.contact_lat)
    long = float(contact.contact_long)
    selected_track = int(getattr(contact, "track_number", 0) or 0)
    if ownship_control_initialized_track != selected_track:
        ownship_control_initialized_track = selected_track
        ownship_current_speed = float(getattr(contact, "speed", 0.0) or 0.0)
        ownship_current_heading = float(getattr(contact, "bearing", ownship_current_heading) or ownship_current_heading) % 360
        ownship_current_depth = float(getattr(contact, "depth", 0.0) or 0.0)
        ownship_commanded_heading = ownship_current_heading
        ownship_commanded_speed = ownship_current_speed
        ownship_commanded_depth = ownship_current_depth if player_is_submarine() else 0.0
        nav_heading_entry.set_text(f"{ownship_commanded_heading:03.0f}")
        nav_speed_entry.set_text(f"{ownship_commanded_speed:.1f}")
        nav_depth_entry.set_text(f"{ownship_commanded_depth:.0f}")

    if ownship_route_active and ownship_route_index < len(ownship_route_waypoints):
        target_world = ownship_route_waypoints[ownship_route_index]
        target_lat, target_lon = pix_to_latlong(target_world.x, target_world.y)
        distance_nm = haversine(lat, long, target_lat, target_lon)
        if distance_nm <= max(0.05, ownship_current_speed / 720.0):
            ownship_route_index += 1
            if ownship_route_index >= len(ownship_route_waypoints):
                ownship_route_active = False
                ownship_route_status = "Route complete"
                nav_route_status_label.set_text(ownship_route_status)
            else:
                target_world = ownship_route_waypoints[ownship_route_index]
                target_lat, target_lon = pix_to_latlong(target_world.x, target_world.y)
        if ownship_route_active:
            ownship_commanded_heading = haversine_bearing(lat, long, target_lat, target_lon)
            nav_heading_entry.set_text(f"{ownship_commanded_heading:03.0f}")

    max_turn = (1.5 if player_is_surface_ship() else 1.0) * max(0.0, dt_seconds)
    previous_heading = ownship_current_heading
    ownship_current_heading = approach_bearing(ownship_current_heading, ownship_commanded_heading, max_turn)
    if dt_seconds > 0:
        delta = (ownship_current_heading - previous_heading + 180) % 360 - 180
        ownship_heading_rate_dps = delta / dt_seconds
    ownship_last_heading = previous_heading
    ownship_current_speed = approach_value(ownship_current_speed, ownship_commanded_speed, (0.8 if player_is_surface_ship() else 0.5) * max(0.0, dt_seconds))
    ownship_current_depth = approach_value(ownship_current_depth, ownship_commanded_depth if player_is_submarine() else 0.0, 35.0 * max(0.0, dt_seconds))

    contact.bearing = ownship_current_heading
    contact.speed = ownship_current_speed
    contact.depth = ownship_current_depth if player_is_submarine() else 0.0
    contact.detected = True
    hdg = ownship_current_heading
    alt = -ownship_current_depth if player_is_submarine() else 0.0

def update_permanent_ownship_sonar():
    if not player_is_ship_or_sub() or lat is None or long is None:
        return
    controlled = selected_ownship_control_contact()
    sonar_range_nm = 14.0 if player_is_submarine() else 8.0
    for contact in contacts:
        if contact is controlled or is_dicass_ping_contact(contact):
            continue
        distance_nm = haversine(float(lat), float(long), contact.contact_lat, contact.contact_long)
        if distance_nm <= sonar_range_nm:
            contact.detected = True
            contact.last_ownship_sonar_range_nm = distance_nm
            contact.last_ownship_sonar_bearing = haversine_bearing(float(lat), float(long), contact.contact_lat, contact.contact_long)

def draw_nav_display(surface):
    surface.fill((2, 8, 14))
    title_font = pygame.font.SysFont(None, scaled_font_size(28))
    text_font = pygame.font.SysFont(None, scaled_font_size(22))
    small_font = pygame.font.SysFont(None, scaled_font_size(18))
    green = (80, 255, 160)
    dim = (80, 120, 130)
    white = (220, 240, 245)
    amber = (255, 220, 120)
    center = (surface.get_width() // 2, 540)
    radius = 190

    pygame.draw.circle(surface, dim, center, radius, 1)
    for deg in range(0, 360, 30):
        rad = math.radians(deg)
        outer = (center[0] + math.sin(rad) * radius, center[1] - math.cos(rad) * radius)
        inner = (center[0] + math.sin(rad) * (radius - 12), center[1] - math.cos(rad) * (radius - 12))
        pygame.draw.line(surface, dim, outer, inner, 1)
        label = small_font.render(f"{deg:03d}", True, white)
        label_pos = (center[0] + math.sin(rad) * (radius + 24) - label.get_width() // 2, center[1] - math.cos(rad) * (radius + 24) - label.get_height() // 2)
        surface.blit(label, label_pos)

    heading_rad = math.radians(ownship_current_heading)
    nose = (center[0] + math.sin(heading_rad) * (radius - 28), center[1] - math.cos(heading_rad) * (radius - 28))
    pygame.draw.line(surface, green, center, nose, 4)
    cmd_rad = math.radians(ownship_commanded_heading)
    cmd = (center[0] + math.sin(cmd_rad) * (radius - 8), center[1] - math.cos(cmd_rad) * (radius - 8))
    pygame.draw.line(surface, amber, center, cmd, 2)

    zulu = datetime.datetime.utcnow().strftime("%Y-%m-%d %H:%M:%SZ")
    mode = f"{dropdown_value(MULTIPLAYER_PLAYER_TYPE).upper()} {dropdown_value(MULTIPLAYER_TEAM).upper()}"
    surface.blit(title_font.render(f"NAVIGATION  {mode}", True, green), (28, 28))
    surface.blit(text_font.render(f"Zulu: {zulu}", True, white), (28, 62))
    lines = [
        f"Current heading: {ownship_current_heading:06.2f}",
        f"Heading rate:    {ownship_heading_rate_dps:+.2f} deg/s",
        f"Current speed:   {ownship_current_speed:.1f} kt",
    ]
    if player_is_submarine():
        lines.append(f"Current depth:   {ownship_current_depth:.0f} ft")
    for idx, line in enumerate(lines):
        surface.blit(text_font.render(line, True, white), (28, 286 + idx * 30))

    route_y = 785
    if ownship_route_active and ownship_route_index < len(ownship_route_waypoints) and lat is not None and long is not None:
        wpt = ownship_route_waypoints[ownship_route_index]
        wpt_lat, wpt_lon = pix_to_latlong(wpt.x, wpt.y)
        dist_nm = haversine(float(lat), float(long), wpt_lat, wpt_lon)
        eta_min = (dist_nm / max(ownship_current_speed, 0.1)) * 60.0
        bearing = haversine_bearing(float(lat), float(long), wpt_lat, wpt_lon)
        route_lines = [
            f"Waypoint {ownship_route_index + 1}/{len(ownship_route_waypoints)}  {latlon_to_navigraph(wpt_lat, wpt_lon)}",
            f"Bearing {bearing:03.0f}  Distance {dist_nm:.2f} NM  ETA {eta_min:.1f} min",
        ]
    else:
        route_lines = [ownship_route_status]
    for idx, line in enumerate(route_lines):
        surface.blit(text_font.render(line, True, amber), (28, route_y + idx * 30))



def snap_search_anchor_latlon(lat_value, lon_value):
    own_lat, own_lon = current_aircraft_latlon_for_search()
    range_nm = haversine(own_lat, own_lon, lat_value, lon_value)
    bearing_deg = haversine_bearing(own_lat, own_lon, lat_value, lon_value)
    snapped_range_nm = round(range_nm * 2) / 2
    if snapped_range_nm <= 0:
        return own_lat, own_lon
    return destination_from_bearing(own_lat, own_lon, bearing_deg, snapped_range_nm)


def set_search_anchor_from_latlon(lat_value, lon_value, status_prefix="Datum"):
    global search_pattern_anchor_world

    snapped_lat, snapped_lon = snap_search_anchor_latlon(lat_value, lon_value)
    search_pattern_anchor_world = pygame.Vector2(latlong_to_pix(snapped_lat, snapped_lon))
    search_pattern_datum_lat_entry.set_text(f"{snapped_lat:.5f}")
    search_pattern_datum_lon_entry.set_text(f"{snapped_lon:.5f}")
    own_lat, own_lon = current_aircraft_latlon_for_search()
    offset_bearing = haversine_bearing(own_lat, own_lon, snapped_lat, snapped_lon)
    offset_range = haversine(own_lat, own_lon, snapped_lat, snapped_lon)
    search_pattern_offset_bearing_entry.set_text(f"{int(round(offset_bearing)) % 360:03d}")
    search_pattern_offset_range_entry.set_text(f"{offset_range:.1f}")
    search_pattern_status_label.set_text(f"{status_prefix} {latlon_to_navigraph(snapped_lat, snapped_lon)}")


def apply_typed_search_datum():
    lat_text = search_pattern_datum_lat_entry.get_text().strip()
    lon_text = search_pattern_datum_lon_entry.get_text().strip()

    if lon_text == "":
        parsed = parse_navigraph_waypoints(lat_text)
        if parsed:
            lat_value, lon_value = pix_to_latlong(parsed[0].x, parsed[0].y)
            set_search_anchor_from_latlon(lat_value, lon_value, "Typed")
            generate_search_pattern()
            return

    try:
        lat_value = float(lat_text)
        lon_value = float(lon_text)
    except ValueError:
        search_pattern_status_label.set_text("Enter decimal lat/lon or one Navigraph point.")
        return

    if not (-90 <= lat_value <= 90 and -180 <= lon_value <= 180):
        search_pattern_status_label.set_text("Datum outside valid lat/lon range.")
        return

    set_search_anchor_from_latlon(lat_value, lon_value, "Typed")
    generate_search_pattern()


def apply_search_offset_from_aircraft():
    bearing_deg = search_pattern_float(search_pattern_offset_bearing_entry, 0.0, 0.0, 359.9)
    range_nm = search_pattern_float(search_pattern_offset_range_entry, 0.0, 0.0, 500.0)
    range_nm = round(range_nm * 2) / 2
    search_pattern_offset_range_entry.set_text(f"{range_nm:.1f}")
    own_lat, own_lon = current_aircraft_latlon_for_search()
    datum_lat, datum_lon = destination_from_bearing(own_lat, own_lon, bearing_deg, range_nm)
    set_search_anchor_from_latlon(datum_lat, datum_lon, "Offset")
    generate_search_pattern()


def save_search_pattern_reference():
    if not search_pattern_waypoints:
        search_pattern_status_label.set_text("No search to save.")
        return
    search_pattern_saved_references.append({
        "anchor": pygame.Vector2(search_pattern_anchor_world) if search_pattern_anchor_world is not None else None,
        "waypoints": [pygame.Vector2(point) for point in search_pattern_waypoints],
        "buoy_points": [pygame.Vector2(point) for point in search_pattern_buoy_points],
        "label": f"REF {len(search_pattern_saved_references) + 1}"
    })
    search_pattern_status_label.set_text(f"Saved {len(search_pattern_saved_references)} radar ref.")


def clear_search_pattern_references():
    search_pattern_saved_references.clear()
    search_pattern_status_label.set_text("Cleared saved search refs.")


def update_search_pattern_output():
    global search_pattern_output_string

    output_tokens = []
    for waypoint in search_pattern_waypoints:
        lat_value, lon_value = pix_to_latlong(waypoint.x, waypoint.y)
        output_tokens.append(latlon_to_navigraph(lat_value, lon_value))
    search_pattern_output_string = " ".join(output_tokens)
    search_pattern_output_entry.set_text(search_pattern_output_string)


def generate_buoy_points_for_search_pattern(spacing_nm):
    spacing_nm = max(0.1, float(spacing_nm or 0.1))
    points = []
    for index in range(0, len(search_pattern_waypoints) - 1, 2):
        start = pygame.Vector2(search_pattern_waypoints[index])
        end = pygame.Vector2(search_pattern_waypoints[index + 1])
        start_lat, start_lon = pix_to_latlong(start.x, start.y)
        end_lat, end_lon = pix_to_latlong(end.x, end.y)
        leg_nm = max(0.0, haversine(start_lat, start_lon, end_lat, end_lon))
        if leg_nm <= 0.01:
            continue
        steps = max(1, int(math.floor(leg_nm / spacing_nm)))
        for step in range(steps + 1):
            fraction = min(1.0, (step * spacing_nm) / leg_nm)
            point = start.lerp(end, fraction)
            if not points or pygame.Vector2(points[-1]).distance_to(point) > 1.0:
                points.append(point)
    return points


def refresh_search_pattern_buoy_points():
    global search_pattern_buoy_points
    spacing_nm = search_pattern_float(search_pattern_buoy_spacing_entry, 2.0, 0.1, 60.0)
    search_pattern_buoy_spacing_entry.set_text(f"{spacing_nm:.1f}")
    search_pattern_buoy_points = generate_buoy_points_for_search_pattern(spacing_nm)


def search_pattern_auto_buoy_selection():
    selection = str(sonobuoy_dropdown.selected_option)
    if selection in ("SSQ-53D(DIFAR)", "SSQ-62(DICASS)", "SSQ-36B(XBT)"):
        return selection
    return "SSQ-53D(DIFAR)"


def dispatch_search_pattern_launch_action(action):
    if multiplayer_role == "JOIN":
        sock = ensure_multiplayer_socket()
        if sock is None or multiplayer_host_seen is None:
            return False
        try:
            sock.sendto(json.dumps(action).encode("utf-8"), ("255.255.255.255", MULTIPLAYER_PORT))
            return True
        except OSError as exc:
            print(f"[MP] auto buoy launch request failed: {exc}")
            return False
    apply_multiplayer_launch_action(action)
    return True


def build_auto_buoy_pending_drops():
    refresh_search_pattern_buoy_points()
    selection = search_pattern_auto_buoy_selection()
    update_multiplayer_channel_assignments()
    update_launch_channel_pool()
    start_ch, end_ch = player_channel_range
    used_channels = deployed_sonobuoy_channels()
    available_channels = [channel for channel in range(start_ch, end_ch + 1) if channel not in used_channels]
    pending = []
    for point, channel in zip(search_pattern_buoy_points, available_channels):
        pending.append({
            "point": pygame.Vector2(point),
            "channel": int(channel),
            "selection": selection,
            "depth": float(depth),
            "dropped": False,
        })
    return pending


def set_auto_buoy_enabled(enabled):
    global auto_buoy_enabled, auto_buoy_pending_drops
    if enabled:
        if not search_pattern_waypoints:
            search_pattern_status_label.set_text("Generate a pattern first.")
            auto_buoy_enabled = False
            auto_buoy_pending_drops = []
        else:
            auto_buoy_pending_drops = build_auto_buoy_pending_drops()
            if not auto_buoy_pending_drops:
                search_pattern_status_label.set_text("No auto buoy points/channels.")
                auto_buoy_enabled = False
            else:
                auto_buoy_enabled = True
                search_pattern_status_label.set_text(f"Auto buoy armed: {len(auto_buoy_pending_drops)} pending.")
    else:
        auto_buoy_enabled = False
        auto_buoy_pending_drops = []
        search_pattern_status_label.set_text("Auto buoy off.")
    sync_auto_buoy_button_style()


def auto_drop_search_pattern_buoys():
    set_auto_buoy_enabled(not auto_buoy_enabled)


def launch_auto_buoy_drop(drop):
    point = pygame.Vector2(drop["point"])
    launch_lat, launch_lon = pix_to_latlong(point.x, point.y)
    action = {
        "kind": "vasw_mp",
        "action": "launch",
        "id": MULTIPLAYER_ID,
        "callsign": MULTIPLAYER_CALLSIGN,
        "selection": drop["selection"],
        "depth": float(drop["depth"]),
        "channel": int(drop["channel"]),
        "lat": float(launch_lat),
        "lon": float(launch_lon),
        "hdg": float(hdg or 0),
        "seeker_mode": selected_torpedo_mode,
        "target_frequency": selected_torpedo_frequency,
    }
    return dispatch_search_pattern_launch_action(action)


def update_auto_buoy_system():
    global auto_buoy_enabled, auto_buoy_last_drop_time
    if not auto_buoy_enabled or not auto_buoy_pending_drops or not hasattr(player_pos, "x"):
        return
    now = time.time()
    if now - auto_buoy_last_drop_time < auto_buoy_min_drop_interval:
        return
    player_world = pygame.Vector2(latlong_to_pix(player_pos.x, player_pos.y))
    for drop in auto_buoy_pending_drops:
        if drop.get("dropped"):
            continue
        distance_nm = world_distance_nm(player_world, pygame.Vector2(drop["point"]))
        drop["distance_nm"] = distance_nm
        if distance_nm <= auto_buoy_trigger_nm:
            if launch_auto_buoy_drop(drop):
                drop["dropped"] = True
                auto_buoy_last_drop_time = now
                update_launch_channel_pool()
                for slot in spectrogram_slot_array:
                    slot.update_ui()
                print(f"AUTO BUOY dropped {drop['selection']} CH {drop['channel']} at {distance_nm:.2f} NM")
            break
    if all(drop.get("dropped") for drop in auto_buoy_pending_drops):
        auto_buoy_enabled = False
        search_pattern_status_label.set_text("Auto buoy complete.")
    sync_auto_buoy_button_style()


def generate_parallel_sweep(anchor_lat, anchor_lon, heading_deg, leg_length_nm, spacing_nm, track_count):
    waypoints = []
    track_count = max(2, track_count)
    half_length = max(0.1, leg_length_nm) / 2
    spacing_nm = max(0.1, spacing_nm)
    cross_bearing = (heading_deg + 90) % 360

    for index in range(track_count):
        offset_nm = (index - (track_count - 1) / 2) * spacing_nm
        center_lat, center_lon = destination_from_bearing(anchor_lat, anchor_lon, cross_bearing, offset_nm)
        start_lat, start_lon = destination_from_bearing(center_lat, center_lon, (heading_deg + 180) % 360, half_length)
        end_lat, end_lon = destination_from_bearing(center_lat, center_lon, heading_deg, half_length)
        if index % 2:
            start_lat, start_lon, end_lat, end_lon = end_lat, end_lon, start_lat, start_lon
        waypoints.append(pygame.Vector2(latlong_to_pix(start_lat, start_lon)))
        waypoints.append(pygame.Vector2(latlong_to_pix(end_lat, end_lon)))

    return waypoints


def generate_parallel_sweep_from_start(start_lat, start_lon, heading_deg, leg_length_nm, spacing_nm, track_count):
    waypoints = []
    track_count = max(2, track_count)
    leg_length_nm = max(0.1, leg_length_nm)
    spacing_nm = max(0.1, spacing_nm)
    cross_bearing = (heading_deg + 90) % 360

    far_start_lat, far_start_lon = destination_from_bearing(start_lat, start_lon, heading_deg, leg_length_nm)
    for index in range(track_count):
        if index % 2 == 0:
            line_start_lat, line_start_lon = destination_from_bearing(start_lat, start_lon, cross_bearing, index * spacing_nm)
            line_end_lat, line_end_lon = destination_from_bearing(line_start_lat, line_start_lon, heading_deg, leg_length_nm)
        else:
            line_start_lat, line_start_lon = destination_from_bearing(far_start_lat, far_start_lon, cross_bearing, index * spacing_nm)
            line_end_lat, line_end_lon = destination_from_bearing(start_lat, start_lon, cross_bearing, index * spacing_nm)

        waypoints.append(pygame.Vector2(latlong_to_pix(line_start_lat, line_start_lon)))
        waypoints.append(pygame.Vector2(latlong_to_pix(line_end_lat, line_end_lon)))

    return waypoints


def generate_search_pattern():
    global search_pattern_waypoints

    if search_pattern_anchor_world is None:
        search_pattern_status_label.set_text("Place a datum with both mouse buttons first.")
        return

    anchor_lat, anchor_lon = pix_to_latlong(search_pattern_anchor_world.x, search_pattern_anchor_world.y)
    heading_deg = search_pattern_float(search_pattern_heading_entry, hdg, 0, 359.9)
    leg_length_nm = search_pattern_float(search_pattern_length_entry, 12.0, 0.1, 250.0)
    spacing_nm = search_pattern_float(search_pattern_spacing_entry, 2.0, 0.1, 60.0)
    count = search_pattern_int(search_pattern_count_entry, 6, 2, 60)

    if search_pattern_anchor_mode == "CENTER":
        search_pattern_waypoints = generate_parallel_sweep(anchor_lat, anchor_lon, heading_deg, leg_length_nm, spacing_nm, count)
    else:
        search_pattern_waypoints = generate_parallel_sweep_from_start(anchor_lat, anchor_lon, heading_deg, leg_length_nm, spacing_nm, count)

    update_search_pattern_output()
    refresh_search_pattern_buoy_points()
    search_pattern_status_label.set_text(f"{len(search_pattern_waypoints)} pts, {len(search_pattern_buoy_points)} buoys.")


def import_search_pattern_waypoints():
    global search_pattern_waypoints, search_pattern_anchor_world

    imported = parse_navigraph_waypoints(search_pattern_import_entry.get_text())
    if not imported:
        search_pattern_status_label.set_text("No valid Navigraph waypoints found.")
        return
    imported_lat, imported_lon = pix_to_latlong(imported[0].x, imported[0].y)
    set_search_anchor_from_latlon(imported_lat, imported_lon, "Imported")
    snap_delta = search_pattern_anchor_world - imported[0]
    search_pattern_waypoints = [pygame.Vector2(point) + snap_delta for point in imported]
    update_search_pattern_output()
    refresh_search_pattern_buoy_points()
    search_pattern_status_label.set_text(f"Imported {len(imported)} pts, {len(search_pattern_buoy_points)} buoys.")


def copy_search_pattern_waypoints():
    if not search_pattern_output_string:
        search_pattern_status_label.set_text("No waypoints to copy.")
        return
    try:
        if hasattr(pygame.scrap, "get_init") and not pygame.scrap.get_init():
            pygame.scrap.init()
        pygame.scrap.put(pygame.SCRAP_TEXT, search_pattern_output_string.encode("utf-8"))
        search_pattern_status_label.set_text("Copied waypoints.")
    except Exception:
        search_pattern_status_label.set_text("Copy failed; select the waypoint text.")


def copy_latlon_at_screen_pos(screen_pos):
    if in_menu:
        return False

    internal_pos = internal_mouse_pos(screen_pos)
    world_pos = right_display_pos_to_world(internal_pos)
    if world_pos is None:
        return False

    lat_value, lon_value = pix_to_latlong(world_pos.x, world_pos.y)
    decimal_text = f"{lat_value:.5f}, {lon_value:.5f}"
    nav_text = latlon_to_navigraph(lat_value, lon_value)
    clipboard_text = f"{decimal_text}\n{nav_text}"

    try:
        if hasattr(pygame.scrap, "get_init") and not pygame.scrap.get_init():
            pygame.scrap.init()
        pygame.scrap.put(pygame.SCRAP_TEXT, clipboard_text.encode("utf-8"))
        print(f"Copied lat/lon: {decimal_text} ({nav_text})")
        if search_pattern_panel.visible:
            search_pattern_status_label.set_text(f"Copied {nav_text}")
        return True
    except Exception as exc:
        print(f"Copy lat/lon failed: {exc}")
        return False


def open_search_pattern_panel(world_pos):
    anchor_lat, anchor_lon = pix_to_latlong(world_pos.x, world_pos.y)
    set_search_anchor_from_latlon(anchor_lat, anchor_lon, "Datum")
    search_pattern_heading_entry.set_text(f"{int(round(hdg)) % 360:03d}")
    set_search_pattern_panel_visible(True)
    generate_search_pattern()


def draw_search_pattern_buoy_points(surface, buoy_points, colour=(120, 255, 160), label_prefix="B"):
    for index, point in enumerate(buoy_points):
        screen_pos = world_to_right_display_pos(point)
        if screen_pos is None:
            continue
        screen_pos = pygame.Vector2(screen_pos)
        pygame.draw.circle(surface, colour, screen_pos, 4, 1)
        pygame.draw.line(surface, colour, (screen_pos.x - 5, screen_pos.y), (screen_pos.x + 5, screen_pos.y), 1)
        pygame.draw.line(surface, colour, (screen_pos.x, screen_pos.y - 5), (screen_pos.x, screen_pos.y + 5), 1)
        label = font.render(f"{label_prefix}{index + 1}", False, colour)
        surface.blit(label, (screen_pos.x + 6, screen_pos.y + 4))


def draw_search_pattern_route(surface, waypoints, anchor_world=None, colour=(130, 220, 230), dim_colour=(70, 120, 125), label_prefix=""):
    screen_points = []
    for waypoint in waypoints:
        screen_pos = world_to_right_display_pos(waypoint)
        if screen_pos is None:
            screen_points.append(None)
        else:
            screen_points.append(pygame.Vector2(screen_pos))

    for index in range(1, len(screen_points)):
        if screen_points[index - 1] is None or screen_points[index] is None:
            continue
        draw_dotted_line(surface, colour, screen_points[index - 1], screen_points[index], 2, dash_length=8, gap_length=5)

    for index, screen_pos in enumerate(screen_points):
        if screen_pos is None:
            continue
        pygame.draw.circle(surface, colour, screen_pos, 5, 1)
        label = font.render(f"{label_prefix}{index + 1}", False, colour)
        surface.blit(label, (screen_pos.x + 7, screen_pos.y - 7))

    if anchor_world is not None:
        anchor_screen = world_to_right_display_pos(anchor_world)
        if anchor_screen is not None:
            pygame.draw.circle(surface, dim_colour, anchor_screen, 11, 1)
            pygame.draw.line(surface, dim_colour, (anchor_screen[0] - 14, anchor_screen[1]), (anchor_screen[0] + 14, anchor_screen[1]), 1)
            pygame.draw.line(surface, dim_colour, (anchor_screen[0], anchor_screen[1] - 14), (anchor_screen[0], anchor_screen[1] + 14), 1)


def draw_search_pattern(surface):
    for reference in search_pattern_saved_references:
        draw_search_pattern_route(
            surface,
            reference["waypoints"],
            reference.get("anchor"),
            colour=(230, 210, 120),
            dim_colour=(140, 120, 70),
            label_prefix=""
        )
        draw_search_pattern_buoy_points(surface, reference.get("buoy_points", []), colour=(180, 220, 120), label_prefix="R")

    if search_pattern_waypoints:
        draw_search_pattern_route(surface, search_pattern_waypoints, search_pattern_anchor_world)
        draw_search_pattern_buoy_points(surface, search_pattern_buoy_points)
    if auto_buoy_pending_drops:
        pending_points = [drop["point"] for drop in auto_buoy_pending_drops if not drop.get("dropped")]
        dropped_points = [drop["point"] for drop in auto_buoy_pending_drops if drop.get("dropped")]
        draw_search_pattern_buoy_points(surface, dropped_points, colour=(80, 120, 80), label_prefix="D")
        draw_search_pattern_buoy_points(surface, pending_points, colour=(255, 230, 90), label_prefix="A")


def draw_scale_bar(surface, map_layer):

    surf_width, surf_height = surface.get_size()

    start_x = 30
    start_y = surf_height - 30
    pixel_length = 100

    # --- Convert screen → world manually ---
    def screen_to_world(screen_pos):
        sx, sy = screen_pos

        world_x = ((sx - surf_width/2) / map_layer.zoom) + map_centre_x
        world_y = ((sy - surf_height/2) / map_layer.zoom) + map_centre_y

        return world_x, world_y

    world1 = screen_to_world((start_x, start_y))
    world2 = screen_to_world((start_x + pixel_length, start_y))

    lat1, lon1 = pix_to_latlong(*world1)
    lat2, lon2 = pix_to_latlong(*world2)

    distance_nm = haversine(lat1, lon1, lat2, lon2)

    nice_values = [0.1, 0.25, 0.5, 1, 2, 5, 10, 20, 50]
    display_nm = min(nice_values, key=lambda x: abs(x - distance_nm))

    scale_pixels = pixel_length * (display_nm / distance_nm)

    end_x = start_x + scale_pixels

    pygame.draw.line(surface, (255,255,255), (start_x, start_y), (end_x, start_y), 3)
    pygame.draw.line(surface, (255,255,255), (start_x, start_y-6), (start_x, start_y+6), 2)
    pygame.draw.line(surface, (255,255,255), (end_x, start_y-6), (end_x, start_y+6), 2)

    font_small = pygame.font.SysFont(None, scaled_font_size(20))
    label = font_small.render(f"{display_nm} NM", True, (255,255,255))
    surface.blit(label, (start_x, start_y - 22))


def radar_point_from_world(world_pos, ownship_world_pos, center, pixels_per_nm, rotation_deg):
    target_lat, target_lon = pix_to_latlong(world_pos.x, world_pos.y)
    own_lat, own_lon = pix_to_latlong(ownship_world_pos.x, ownship_world_pos.y)
    return radar_point_from_latlon(target_lat, target_lon, own_lat, own_lon, center, pixels_per_nm, rotation_deg)


def ensure_multiplayer_socket():
    global multiplayer_socket

    if multiplayer_socket is not None:
        return multiplayer_socket

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.setblocking(False)
        sock.bind(("", MULTIPLAYER_PORT))
        multiplayer_socket = sock
        print(f"[MP] UDP multiplayer listening on port {MULTIPLAYER_PORT} as {MULTIPLAYER_CALLSIGN}")
    except OSError as exc:
        multiplayer_socket = None
        print(f"[MP] multiplayer socket failed: {exc}")

    return multiplayer_socket


def multiplayer_host_packet():
    return {
        "kind": "vASW-host",
        "id": MULTIPLAYER_ID,
        "callsign": MULTIPLAYER_CALLSIGN,
        "aircraft_type": MULTIPLAYER_AIRCRAFT_TYPE,
        "role": multiplayer_role,
        "password_required": multiplayer_role == "SERVER",
        "timestamp": time.time()
    }


def tone_to_mp(tone):
    return {
        "freq": float(getattr(tone, "freq", 0.0)),
        "db": float(getattr(tone, "db", 0.0)),
        "label": str(getattr(tone, "label", "Tone")),
        "harmonics": int(getattr(tone, "harmonics", 1)),
        "harmonic_drop": float(getattr(tone, "harmonic_drop", 6.0))
    }


def contact_to_mp(contact):
    return {
        "track_number": int(getattr(contact, "track_number", 0) or 0),
        "name": str(getattr(contact, "name", "Contact")),
        "lat": float(getattr(contact, "contact_lat", 0.0)),
        "lon": float(getattr(contact, "contact_long", 0.0)),
        "speed": float(getattr(contact, "speed", 0.0)),
        "depth": float(getattr(contact, "depth", 0.0)),
        "bearing": float(getattr(contact, "bearing", 0.0)),
        "detected": bool(getattr(contact, "detected", False)),
        "broadcasting": bool(getattr(contact, "broadcasting", False)),
        "internal_type": str(getattr(contact, "internal_type", "Unknown")),
        "internal_class": str(getattr(contact, "internal_class", "Unknown")),
        "classification_type": str(getattr(contact, "classification_type", "Unknown")),
        "classification_class": str(getattr(contact, "classification_class", "Unknown")),
        "identity_status": str(getattr(contact, "identity_status", "P")),
        "operator_classified": bool(getattr(contact, "operator_classified", False)),
        "route": ship_route_config_from_contact(contact),
        "route_index": int(getattr(contact, "route_index", 0) or 0),
        "route_active": bool(getattr(contact, "route_active", False)),
        "route_status": str(getattr(contact, "route_status", "No route")),
        "tones": [tone_to_mp(tone) for tone in getattr(contact, "tones", [])],
        "mp_source_id": str(getattr(contact, "mp_source_id", "")),
        "mp_source_track": int(getattr(contact, "mp_source_track", getattr(contact, "track_number", 0)) or 0)
    }


def contact_from_mp_item(item, preserve_track=True):
    tones = [
        Tone(
            freq=float(tone.get("freq", 0.0)),
            db=float(tone.get("db", 0.0)),
            label=str(tone.get("label", "Tone")),
            harmonics=int(tone.get("harmonics", 1)),
            harmonic_drop=float(tone.get("harmonic_drop", 6.0))
        )
        for tone in item.get("tones", [])
    ]
    contact = Contact(
        name=str(item.get("name", "Contact")),
        tones=tones,
        contact_lat=float(item.get("lat", 0.0)),
        contact_long=float(item.get("lon", 0.0)),
        speed=float(item.get("speed", 0.0)),
        depth=float(item.get("depth", 0.0)),
        bearing=float(item.get("bearing", 0.0))
    )
    if preserve_track:
        contact.track_number = int(item.get("track_number", contact.track_number))
    contact.detected = bool(item.get("detected", False))
    contact.broadcasting = bool(item.get("broadcasting", False))
    contact.internal_type = str(item.get("internal_type", "Unknown"))
    contact.internal_class = str(item.get("internal_class", "Unknown"))
    contact.classification_type = str(item.get("classification_type", "Unknown"))
    contact.classification_class = str(item.get("classification_class", "Unknown"))
    contact.identity_status = str(item.get("identity_status", "P"))
    contact.operator_classified = bool(item.get("operator_classified", False))
    contact.mp_source_id = str(item.get("mp_source_id", ""))
    contact.mp_source_track = int(item.get("mp_source_track", item.get("track_number", contact.track_number)) or 0)
    route_config = item.get("route")
    if route_config is not None and configure_ship_route(contact, route_config, now=float(item.get("timestamp", time.time()) or time.time())):
        contact.route_index = int(item.get("route_index", getattr(contact, "route_index", 0)) or 0)
        contact.route_active = bool(item.get("route_active", getattr(contact, "route_active", False)))
        contact.route_status = str(item.get("route_status", getattr(contact, "route_status", "No route")))
    return contact


def local_contacts_for_contribution():
    return [
        contact for contact in contacts
        if hasattr(contact, "tones")
        and not is_dicass_ping_contact(contact)
        and not getattr(contact, "mp_from_server", False)
    ]


def multiplayer_contact_contribution_packet():
    return {
        "kind": "vASW-action",
        "id": MULTIPLAYER_ID,
        "action": "contacts",
        "password": multiplayer_contact_password,
        "callsign": MULTIPLAYER_CALLSIGN,
        "timestamp": time.time(),
        "contacts": [
            dict(contact_to_mp(contact), mp_source_id=MULTIPLAYER_ID, mp_source_track=int(getattr(contact, "track_number", 0) or 0))
            for contact in local_contacts_for_contribution()
        ]
    }


def apply_multiplayer_contact_contribution(packet):
    global multiplayer_last_state_broadcast
    if not multiplayer_contact_password_ok(packet.get("password", "")):
        print(f"[MP] contact contribution rejected from {packet.get('callsign', packet.get('id', 'MP'))}: bad password")
        return

    source_id = str(packet.get("id", ""))
    updated = 0
    created = 0
    for item in packet.get("contacts", []):
        try:
            contact = contact_from_mp_item(item, preserve_track=False)
            contact.mp_source_id = source_id
            contact.mp_source_track = int(item.get("track_number", item.get("mp_source_track", 0)) or 0)
            contact.mp_contributed = True
            contact.mp_from_server = False
        except (TypeError, ValueError):
            continue

        existing = next(
            (
                candidate for candidate in contacts
                if str(getattr(candidate, "mp_source_id", "")) == source_id
                and int(getattr(candidate, "mp_source_track", -1) or -1) == contact.mp_source_track
            ),
            None
        )
        if existing is None:
            contacts.append(contact)
            created += 1
        else:
            contact.track_number = existing.track_number
            contacts[contacts.index(existing)] = contact
            updated += 1

    if created or updated:
        multiplayer_last_state_broadcast = 0.0
        print(f"[MP] accepted contact contribution from {packet.get('callsign', source_id)}: +{created}, updated {updated}")


def send_multiplayer_contact_contribution():
    if multiplayer_role != "JOIN" or multiplayer_host_seen is None or not multiplayer_contact_password_ok():
        return False
    if not local_contacts_for_contribution():
        return False
    sock = ensure_multiplayer_socket()
    if sock is None:
        return False
    try:
        sock.sendto(json.dumps(multiplayer_contact_contribution_packet()).encode("utf-8"), ("255.255.255.255", MULTIPLAYER_PORT))
        return True
    except OSError as exc:
        print(f"[MP] contact contribution failed: {exc}")
        return False

def buoy_to_mp(buoy, kind):
    data = {
        "kind": kind,
        "x": float(getattr(buoy, "x", 0.0)),
        "y": float(getattr(buoy, "y", 0.0)),
        "depth": float(getattr(buoy, "depth", 0.0) or 0.0),
        "channel": int(getattr(buoy, "channel", 0) or 0),
        "range_circle": bool(getattr(buoy, "range_circle", True)),
        "bearing_lines_visible": bool(getattr(buoy, "bearing_lines_visible", True))
    }
    if kind == "DICASS":
        data.update({
            "start_khz": float(getattr(buoy, "start_khz", 2.0)),
            "end_khz": float(getattr(buoy, "end_khz", 3.0)),
            "sweep_time": float(getattr(buoy, "sweep_time", 7.0)),
            "source_db": float(getattr(buoy, "source_db", 200.0))
        })
    return data


def torpedo_to_mp(torpedo):
    return {
        "x": float(getattr(torpedo, "pos", pygame.Vector2()).x),
        "y": float(getattr(torpedo, "pos", pygame.Vector2()).y),
        "target_frequency": float(getattr(torpedo, "target_frequency", 0.0)),
        "heading": float(getattr(torpedo, "heading", 0.0)),
        "display_heading": float(getattr(torpedo, "display_heading", 0.0)),
        "seeker_mode": str(getattr(torpedo, "seeker_mode", "PASSIVE")),
        "finished": bool(getattr(torpedo, "finished", False)),
        "detonated": bool(getattr(torpedo, "detonated", False)),
        "age": max(0.0, time.time() - float(getattr(torpedo, "launch_time", time.time())))
    }


def search_reference_to_mp(reference):
    return {
        "label": str(reference.get("label", "REF")),
        "anchor": [float(reference["anchor"].x), float(reference["anchor"].y)] if reference.get("anchor") is not None else None,
        "waypoints": [[float(point.x), float(point.y)] for point in reference.get("waypoints", [])]
    }


def multiplayer_state_packet():
    update_multiplayer_channel_assignments()
    return {
        "kind": "vASW-state",
        "id": MULTIPLAYER_ID,
        "callsign": MULTIPLAYER_CALLSIGN,
        "timestamp": time.time(),
        "channel_assignments": multiplayer_channel_assignments,
        "contacts": [
            contact_to_mp(contact) for contact in contacts
            if hasattr(contact, "tones") and not is_dicass_ping_contact(contact)
        ],
        "sonobuoys": [buoy_to_mp(buoy, "DIFAR") for buoy in sono_array],
        "active_sonobuoys": [buoy_to_mp(buoy, "DICASS") for buoy in active_sono_array],
        "torpedoes": [torpedo_to_mp(torpedo) for torpedo in torp_array],
        "search_references": [search_reference_to_mp(reference) for reference in search_pattern_saved_references]
    }


def deployed_sonobuoy_channels():
    channels = set()
    for buoy in list(sono_array) + list(active_sono_array):
        if is_numeric_channel(getattr(buoy, "channel", None)):
            channels.add(int(buoy.channel))
    for label in xbt_profiles.keys():
        match = re.fullmatch(r"CH\s+(\d+)", str(label).strip(), re.IGNORECASE)
        if match:
            channels.add(int(match.group(1)))
    return channels


def update_channel_range_label():
    start_ch, end_ch = player_channel_range
    channel_range_label.set_text(f"Range: {start_ch}-{end_ch}")


def update_launch_channel_pool():
    global channel_array, channel_names, displayed_channel

    start_ch, end_ch = player_channel_range
    used_channels = deployed_sonobuoy_channels()
    available = [
        channel for channel in range(start_ch, end_ch + 1)
        if channel not in used_channels
    ]
    channel_array = [Channel(channel) for channel in available]
    channel_names = [str(channel) for channel in available]

    if displayed_channel not in available:
        displayed_channel = available[0] if available else 0

    current_channel_label.set_text(f"Channel: {displayed_channel}")
    update_channel_range_label()


def channel_range_for_player(player_id):
    assignment = multiplayer_channel_assignments.get(str(player_id))
    if not assignment:
        return (1, 99)
    try:
        return int(assignment["start"]), int(assignment["end"])
    except (KeyError, TypeError, ValueError):
        return (1, 99)


def first_available_channel_in_range(channel_range):
    start_ch, end_ch = channel_range
    used_channels = deployed_sonobuoy_channels()
    for channel in range(start_ch, end_ch + 1):
        if channel not in used_channels:
            return channel
    return None


def update_multiplayer_channel_assignments():
    global multiplayer_channel_assignments, player_channel_range

    if multiplayer_role == "OFF":
        multiplayer_channel_assignments = {
            MULTIPLAYER_ID: {
                "callsign": MULTIPLAYER_CALLSIGN,
                "start": 1,
                "end": 99
            }
        }
        player_channel_range = (1, 99)
        update_launch_channel_pool()
        return

    if multiplayer_is_host_role():
        participants = [{
            "id": MULTIPLAYER_ID,
            "callsign": MULTIPLAYER_CALLSIGN
        }]
        for peer in multiplayer_peers.values():
            participants.append({
                "id": str(peer.get("id")),
                "callsign": str(peer.get("callsign", "MP"))
            })
        participants.sort(key=lambda item: (item["callsign"].upper(), item["id"]))

        count = max(1, len(participants))
        base = 99 // count
        remainder = 99 % count
        assignments = {}
        next_channel = 1
        for index, participant in enumerate(participants):
            span = base + (1 if index < remainder else 0)
            start_ch = next_channel
            end_ch = min(99, next_channel + span - 1)
            assignments[participant["id"]] = {
                "callsign": participant["callsign"],
                "start": start_ch,
                "end": end_ch
            }
            next_channel = end_ch + 1

        if assignments != multiplayer_channel_assignments:
            multiplayer_channel_assignments = assignments
        player_channel_range = channel_range_for_player(MULTIPLAYER_ID)
        update_launch_channel_pool()
        return

    if multiplayer_role == "JOIN":
        player_channel_range = channel_range_for_player(MULTIPLAYER_ID)
        update_launch_channel_pool()


def ensure_sono_channel_option(channel):
    channel_text = str(channel)
    if "None" not in sono_channel_array:
        sono_channel_array.insert(0, "None")
    if channel_text not in sono_channel_array:
        sono_channel_array.append(channel_text)
    sono_channel_array.sort(key=lambda value: -1 if value == "None" else int(value) if str(value).isdigit() else 9999)


def reserve_sono_channel(channel):
    global channel_array, channel_names, displayed_channel
    try:
        channel_number = int(channel)
    except (TypeError, ValueError):
        return
    channel_array = [ch for ch in channel_array if ch.channel_number != channel_number]
    channel_names = [f"{ch.channel_number}" for ch in channel_array]
    if displayed_channel == channel_number:
        displayed_channel = channel_array[0].channel_number if channel_array else 0
    current_channel_label.set_text(f"Channel: {displayed_channel}")
    update_channel_range_label()


def advance_displayed_channel_after_launch(channel):
    global channel_array, channel_names, displayed_channel
    reserve_sono_channel(channel)
    if channel_array:
        displayed_channel = channel_array[0].channel_number
    else:
        displayed_channel = 0
    current_channel_label.set_text(f"Channel: {displayed_channel}")


def apply_multiplayer_launch_action(action):
    global xbt_counter, latest_xbt_profile, xbt_exists, xbt_panel_selected_label, multiplayer_last_state_broadcast

    try:
        selection = str(action.get("selection", ""))
        launch_depth = float(action.get("depth", depth))
        channel = int(action.get("channel", displayed_channel or 0))
        launch_lat = float(action.get("lat"))
        launch_lon = float(action.get("lon"))
    except (TypeError, ValueError):
        return

    if multiplayer_is_host_role() and selection in ("SSQ-53D(DIFAR)", "SSQ-62(DICASS)", "SSQ-36B(XBT)"):
        update_multiplayer_channel_assignments()
        requester_range = channel_range_for_player(str(action.get("id", "")))
        used_channels = deployed_sonobuoy_channels()
        if (
            channel < requester_range[0] or
            channel > requester_range[1] or
            channel in used_channels
        ):
            replacement = first_available_channel_in_range(requester_range)
            if replacement is None:
                print(f"[MP] launch rejected: no channels free in {requester_range[0]}-{requester_range[1]}")
                return
            print(f"[MP] remapped requested CH {channel} to CH {replacement} for assigned range {requester_range[0]}-{requester_range[1]}")
            channel = replacement

    world_pos = latlong_to_pix(launch_lat, launch_lon)
    spawn_splash(world_pos)
    spawn_msfs_splash(launch_lat, launch_lon)

    if selection == "SSQ-53D(DIFAR)":
        buoy = Sonobuoy(world_pos, launch_depth, sonoD_surface, len(sono_array), channel)
        buoy.depth = launch_depth
        sono_array.append(buoy)
        spectro_array.append(SpectrogramUI(buoy))
        ensure_sono_channel_option(channel)
        reserve_sono_channel(channel)
    elif selection == "SSQ-62(DICASS)":
        buoy = Active_Sonobuoy(world_pos, launch_depth, sonoS_surface, len(active_sono_array), channel, 2, 3, 7, 200)
        active_sono_array.append(buoy)
        ensure_sono_channel_option(channel)
        reserve_sono_channel(channel)
    elif selection == "STINGRAY(TORPEDO)":
        torp_array.append(Torpedo(
            world_pos,
            float(action.get("target_frequency", selected_torpedo_frequency)),
            float(action.get("hdg", hdg or 0)),
            torpedo_surface,
            str(action.get("seeker_mode", selected_torpedo_mode))
        ))
    elif selection == "SSQ-36B(XBT)":
        xbt_counter += 1
        xbt_label = f"CH {channel}"
        xbt_pos = pygame.Vector2(world_pos)
        xbt_profile = XBT(
            temp_surface=surface_temp + random.uniform(-1.2, 1.2),
            temp_seabed=seabed_temp + random.uniform(-0.5, 0.5),
            thermocline_depth=max(180, min(seabed_depth - 250, thermocline_depth + random.uniform(-120, 120))),
            max_depth=seabed_depth,
            position=xbt_pos,
            label=xbt_label
        )
        xbt_profile.update()
        latest_xbt_profile = xbt_profile
        xbt_profiles[xbt_label] = xbt_profile
        xbt_array.append(xbt_pos)
        xbt_exists = True
        xbt_panel_selected_label = xbt_label
        update_xbt_panel_selector()
        reserve_sono_channel(channel)

    for slot in spectrogram_slot_array:
        slot.update_ui()
    multiplayer_last_state_broadcast = 0.0
    print(f"[MP] applied launch request: {selection} CH {channel} at {launch_lat:.5f}, {launch_lon:.5f}")


def current_player_lat_lon_for_mp():
    if hasattr(player_pos, "x") and hasattr(player_pos, "y"):
        return float(player_pos.x), float(player_pos.y)
    if lat is not None and long is not None:
        return float(lat), float(long)
    return None


def contact_by_track_number(track_number):
    try:
        track_number = int(track_number)
    except (TypeError, ValueError):
        return None
    for contact in contacts:
        if int(getattr(contact, "track_number", -1) or -1) == track_number:
            return contact
    return None


def apply_contact_command_to_contact(contact, command, payload):
    global multiplayer_last_state_broadcast
    if contact is None:
        return False
    command = str(command or "")

    if command == "classify":
        contact.operator_classified = True
        if "classification_type" in payload:
            contact.classification_type = str(payload.get("classification_type", "Unknown"))
        if "classification_class" in payload:
            contact.classification_class = str(payload.get("classification_class", "Unknown"))
    elif command == "identity":
        contact.operator_classified = True
        contact.identity_status = str(payload.get("identity_status", getattr(contact, "identity_status", "P")))
    elif command == "country":
        contact.operator_classified = True
        contact.country = str(payload.get("country", getattr(contact, "country", "Unknown")))
    elif command == "lines":
        contact.bearing_lines_hidden = bool(payload.get("bearing_lines_hidden", getattr(contact, "bearing_lines_hidden", False)))
    elif command == "ship_stop":
        request_ship_stop(contact)
    elif command == "ship_heading":
        request_ship_heading(contact, float(payload.get("heading", getattr(contact, "bearing", 0.0)) or 0.0))
    elif command == "ship_speed":
        request_ship_speed(contact, payload.get("speed", getattr(contact, "speed", 0.0)))
    elif command == "ship_resume_route":
        resume_ship_route(contact)
    elif command == "ship_route_speed":
        set_ship_route_speed(contact, payload.get("speed", getattr(contact, "route_speed_kts", getattr(contact, "speed", 0.0))))
    elif command == "ship_route":
        assign_ship_route_from_text(contact, str(payload.get("route_text", "")))
    elif command == "delete":
        return delete_contact(contact)
    elif command == "deck_lock":
        if selected_contact is contact:
            toggle_ship_deck_lock(contact)
        else:
            ensure_ship_command_state(contact)
            contact.manual_deck_lock_active = not bool(getattr(contact, "manual_deck_lock_active", False))
    else:
        return False

    multiplayer_last_state_broadcast = 0.0
    return True


def send_multiplayer_contact_command(command, contact=None, **payload):
    contact = contact or selected_contact
    if multiplayer_role != "JOIN" or multiplayer_host_seen is None or contact is None:
        return False
    sock = ensure_multiplayer_socket()
    if sock is None:
        return False
    packet = {
        "kind": "vASW-action",
        "id": MULTIPLAYER_ID,
        "action": "contact_command",
        "track_number": int(getattr(contact, "track_number", 0) or 0),
        "command": command,
        "payload": payload,
        "timestamp": time.time()
    }
    if multiplayer_contact_password:
        packet["password"] = multiplayer_contact_password
    try:
        sock.sendto(json.dumps(packet).encode("utf-8"), ("255.255.255.255", MULTIPLAYER_PORT))
        return True
    except OSError as exc:
        print(f"[MP] contact command failed: {exc}")
        return False


def apply_multiplayer_contact_command(packet):
    command = str(packet.get("command") or "")
    if command in PROTECTED_CONTACT_COMMANDS and not multiplayer_contact_password_ok(packet.get("password", "")):
        print(f"[MP] rejected protected contact command {command}: bad password")
        return
    contact = contact_by_track_number(packet.get("track_number"))
    payload = packet.get("payload", {})
    if not isinstance(payload, dict):
        payload = {}
    if apply_contact_command_to_contact(contact, command, payload):
        print(f"[MP] applied contact command {packet.get('command')} for track {packet.get('track_number')}")

def send_multiplayer_launch_request():
    if multiplayer_role != "JOIN" or multiplayer_host_seen is None or not multiplayer_can_join_server():
        return False
    launch_latlon = current_player_lat_lon_for_mp()
    if launch_latlon is None:
        return False
    launch_lat, launch_lon = launch_latlon
    sock = ensure_multiplayer_socket()
    if sock is None:
        return False
    action = {
        "kind": "vASW-action",
        "id": MULTIPLAYER_ID,
        "action": "launch",
        "selection": sono_selection,
        "depth": float(depth),
        "channel": int(displayed_channel or 0),
        "lat": launch_lat,
        "lon": launch_lon,
        "hdg": float(hdg or 0),
        "seeker_mode": selected_torpedo_mode,
        "target_frequency": float(selected_torpedo_frequency),
        "password": multiplayer_contact_password,
        "timestamp": time.time()
    }
    try:
        sock.sendto(json.dumps(action).encode("utf-8"), ("255.255.255.255", MULTIPLAYER_PORT))
        print(f"[MP] requested host launch: {sono_selection}")
        return True
    except OSError as exc:
        print(f"[MP] launch request failed: {exc}")
        return False


def apply_multiplayer_state(packet):
    global contacts, sono_array, active_sono_array, torp_array, sono_channel_array, spectro_array
    global multiplayer_channel_assignments, player_channel_range

    old_channel_array = list(sono_channel_array)
    assignments = packet.get("channel_assignments", {})
    if isinstance(assignments, dict):
        cleaned_assignments = {}
        for player_id, assignment in assignments.items():
            try:
                cleaned_assignments[str(player_id)] = {
                    "callsign": str(assignment.get("callsign", "MP")),
                    "start": int(assignment.get("start", 1)),
                    "end": int(assignment.get("end", 99))
                }
            except (AttributeError, TypeError, ValueError):
                continue
        if cleaned_assignments:
            multiplayer_channel_assignments = cleaned_assignments
            player_channel_range = channel_range_for_player(MULTIPLAYER_ID)
    existing_sonos = {
        int(sono.channel): sono for sono in sono_array
        if is_numeric_channel(getattr(sono, "channel", None))
    }
    existing_spectros = {
        int(spectro.sono.channel): spectro for spectro in spectro_array
        if is_numeric_channel(getattr(spectro.sono, "channel", None))
    }
    existing_active_sonos = {
        int(active_sono.channel): active_sono for active_sono in active_sono_array
        if is_numeric_channel(getattr(active_sono, "channel", None))
    }
    rebuilt_contacts = []
    for item in packet.get("contacts", []):
        try:
            tones = [
                Tone(
                    freq=float(tone.get("freq", 0.0)),
                    db=float(tone.get("db", 0.0)),
                    label=str(tone.get("label", "Tone")),
                    harmonics=int(tone.get("harmonics", 1)),
                    harmonic_drop=float(tone.get("harmonic_drop", 6.0))
                )
                for tone in item.get("tones", [])
            ]
            contact = Contact(
                name=str(item.get("name", "Contact")),
                tones=tones,
                contact_lat=float(item.get("lat", 0.0)),
                contact_long=float(item.get("lon", 0.0)),
                speed=float(item.get("speed", 0.0)),
                depth=float(item.get("depth", 0.0)),
                bearing=float(item.get("bearing", 0.0))
            )
            contact.track_number = int(item.get("track_number", contact.track_number))
            contact.detected = bool(item.get("detected", False))
            contact.broadcasting = bool(item.get("broadcasting", False))
            contact.internal_type = str(item.get("internal_type", "Unknown"))
            contact.internal_class = str(item.get("internal_class", "Unknown"))
            contact.classification_type = str(item.get("classification_type", "Unknown"))
            contact.classification_class = str(item.get("classification_class", "Unknown"))
            contact.identity_status = str(item.get("identity_status", "P"))
            contact.operator_classified = bool(item.get("operator_classified", False))
            route_config = item.get("route")
            if route_config is not None and configure_ship_route(contact, route_config, now=float(item.get("timestamp", time.time()) or time.time())):
                contact.route_index = int(item.get("route_index", getattr(contact, "route_index", 0)) or 0)
                contact.route_active = bool(item.get("route_active", getattr(contact, "route_active", False)))
                contact.route_status = str(item.get("route_status", getattr(contact, "route_status", "No route")))
            contact.mp_from_server = True
            contact.mp_source_id = str(item.get("mp_source_id", ""))
            contact.mp_source_track = int(item.get("mp_source_track", item.get("track_number", contact.track_number)) or 0)
            rebuilt_contacts.append(contact)
        except (TypeError, ValueError):
            continue
    contacts = rebuilt_contacts

    sono_array = []
    spectro_array = []
    active_sono_array = []
    synced_channels = set()
    for index, item in enumerate(packet.get("sonobuoys", [])):
        try:
            channel = int(item.get("channel", 0))
            if channel in existing_sonos:
                buoy = existing_sonos[channel]
                buoy.x = float(item.get("x", 0.0))
                buoy.y = float(item.get("y", 0.0))
                buoy.rect.x = buoy.x
                buoy.rect.y = buoy.y
            else:
                buoy = Sonobuoy(
                    (float(item.get("x", 0.0)), float(item.get("y", 0.0))),
                    float(item.get("depth", 0.0)),
                    sonoD_surface,
                    index,
                    channel
                )
            buoy.depth = float(item.get("depth", 0.0))
            buoy.range_circle = bool(item.get("range_circle", False))
            buoy.bearing_lines_visible = bool(item.get("bearing_lines_visible", True))
            sono_array.append(buoy)
            if channel in existing_spectros:
                existing_spectros[channel].sono = buoy
                spectro_array.append(existing_spectros[channel])
            else:
                spectro_array.append(SpectrogramUI(buoy))
            synced_channels.add(str(channel))
        except (TypeError, ValueError):
            continue

    for index, item in enumerate(packet.get("active_sonobuoys", [])):
        try:
            channel = int(item.get("channel", 0))
            if channel in existing_active_sonos:
                buoy = existing_active_sonos[channel]
                buoy.x = float(item.get("x", 0.0))
                buoy.y = float(item.get("y", 0.0))
                buoy.rect.x = buoy.x
                buoy.rect.y = buoy.y
                buoy.start_khz = float(item.get("start_khz", buoy.start_khz))
                buoy.end_khz = float(item.get("end_khz", buoy.end_khz))
                buoy.sweep_time = float(item.get("sweep_time", buoy.sweep_time))
                buoy.source_db = float(item.get("source_db", buoy.source_db))
                buoy.bandwidth = buoy.end_khz - buoy.start_khz
            else:
                buoy = Active_Sonobuoy(
                    (float(item.get("x", 0.0)), float(item.get("y", 0.0))),
                    float(item.get("depth", 0.0)),
                    sonoS_surface,
                    index,
                    channel,
                    float(item.get("start_khz", 2.0)),
                    float(item.get("end_khz", 3.0)),
                    float(item.get("sweep_time", 7.0)),
                    float(item.get("source_db", 200.0))
                )
            buoy.depth = float(item.get("depth", 0.0))
            active_sono_array.append(buoy)
            synced_channels.add(str(channel))
        except (TypeError, ValueError):
            continue

    torp_array = []
    for item in packet.get("torpedoes", []):
        try:
            torpedo = Torpedo(
                (float(item.get("x", 0.0)), float(item.get("y", 0.0))),
                float(item.get("target_frequency", 0.0)),
                float(item.get("heading", 0.0)),
                torpedo_surface,
                str(item.get("seeker_mode", "PASSIVE"))
            )
            torpedo.heading = float(item.get("heading", torpedo.heading))
            torpedo.display_heading = float(item.get("display_heading", torpedo.heading))
            torpedo.finished = bool(item.get("finished", False))
            torpedo.detonated = bool(item.get("detonated", False))
            torpedo.launch_time = time.time() - float(item.get("age", 0.0))
            torp_array.append(torpedo)
        except (TypeError, ValueError):
            continue

    search_pattern_saved_references.clear()
    for item in packet.get("search_references", []):
        try:
            anchor = item.get("anchor")
            search_pattern_saved_references.append({
                "label": str(item.get("label", "REF")),
                "anchor": pygame.Vector2(anchor[0], anchor[1]) if anchor else None,
                "waypoints": [pygame.Vector2(point[0], point[1]) for point in item.get("waypoints", [])]
            })
        except (TypeError, ValueError, IndexError):
            continue

    new_channel_array = ["None"] + sorted(synced_channels, key=lambda value: int(value) if value.isdigit() else -1)
    if new_channel_array != old_channel_array:
        sono_channel_array = new_channel_array
        for slot in spectrogram_slot_array:
            slot.update_ui()
    else:
        sono_channel_array = new_channel_array
    update_multiplayer_channel_assignments()


def multiplayer_altitude_label(altitude_ft):
    try:
        altitude_ft = float(altitude_ft)
    except (TypeError, ValueError):
        return "0FT"
    if abs(altitude_ft) >= 1000:
        return f"{altitude_ft / 1000:.1f}KFT"
    return f"{altitude_ft:.0f}FT"


def multiplayer_team_colour(team):
    team = str(team or "").upper()
    if team == "REDFOR":
        return (255, 90, 90)
    if team == "NEUTRAL":
        return (240, 230, 120)
    return (90, 170, 255)


def multiplayer_aircraft_tag(peer):
    callsign = str(peer.get("callsign", "MP")).strip() or "MP"
    player_type = str(peer.get("player_type", "Aircraft"))
    aircraft_type = str(peer.get("aircraft_type", "")).strip()
    parts = [callsign]
    if player_type == "Aircraft":
        if aircraft_type and aircraft_type.lower() not in callsign.lower():
            parts.append(aircraft_type)
        parts.append(multiplayer_altitude_label(peer.get("alt", 0)))
    else:
        parts.append(player_type.upper())
        parts.append(str(peer.get("team", "BLUFOR")).upper())
        parts.append(f"{float(peer.get('speed', 0) or 0):.0f}KT")
        if player_type == "Submarine":
            parts.append(f"D{float(peer.get('depth', 0) or 0):.0f}")
    return " ".join(parts)


def close_multiplayer_socket():
    global multiplayer_socket
    if multiplayer_socket is not None:
        try:
            multiplayer_socket.close()
        except OSError:
            pass
    multiplayer_socket = None


def multiplayer_packet():
    return {
        "kind": "vASW-peer",
        "id": MULTIPLAYER_ID,
        "role": multiplayer_role,
        "callsign": MULTIPLAYER_CALLSIGN,
        "aircraft_type": MULTIPLAYER_AIRCRAFT_TYPE,
        "player_type": MULTIPLAYER_PLAYER_TYPE,
        "team": MULTIPLAYER_TEAM,
        "lat": float(player_pos.x),
        "lon": float(player_pos.y),
        "hdg": float(hdg or 0),
        "alt": float(alt or 0),
        "depth": float(ownship_current_depth if player_is_submarine() else 0.0),
        "speed": float(ownship_current_speed if player_is_ship_or_sub() else aircraft_groundspeed_kt),
        "password": multiplayer_contact_password,
        "timestamp": time.time()
    }


def update_multiplayer():
    global multiplayer_last_broadcast, multiplayer_last_host_broadcast, multiplayer_last_state_broadcast, multiplayer_host_seen, multiplayer_last_contact_contribution

    if not multiplayer_enabled:
        return

    sock = ensure_multiplayer_socket()
    if sock is None:
        return

    now = time.time()
    if multiplayer_is_host_role() and now - multiplayer_last_host_broadcast >= MULTIPLAYER_BROADCAST_INTERVAL:
        multiplayer_last_host_broadcast = now
        try:
            payload = json.dumps(multiplayer_host_packet()).encode("utf-8")
            sock.sendto(payload, ("255.255.255.255", MULTIPLAYER_PORT))
        except OSError as exc:
            print(f"[MP] host beacon failed: {exc}")

    can_send_aircraft = hasattr(player_pos, "x") and hasattr(player_pos, "y")
    if multiplayer_role == "JOIN" and (multiplayer_host_seen is None or not multiplayer_can_join_server()):
        can_send_aircraft = False

    if can_send_aircraft and now - multiplayer_last_broadcast >= MULTIPLAYER_BROADCAST_INTERVAL:
        multiplayer_last_broadcast = now
        try:
            payload = json.dumps(multiplayer_packet()).encode("utf-8")
            sock.sendto(payload, ("255.255.255.255", MULTIPLAYER_PORT))
        except OSError as exc:
            print(f"[MP] broadcast failed: {exc}")

    if multiplayer_role == "JOIN" and multiplayer_can_join_server() and now - multiplayer_last_contact_contribution >= MULTIPLAYER_CONTACT_CONTRIBUTION_INTERVAL:
        multiplayer_last_contact_contribution = now
        send_multiplayer_contact_contribution()

    if multiplayer_is_host_role() and now - multiplayer_last_state_broadcast >= MULTIPLAYER_STATE_BROADCAST_INTERVAL:
        multiplayer_last_state_broadcast = now
        try:
            payload = json.dumps(multiplayer_state_packet()).encode("utf-8")
            sock.sendto(payload, ("255.255.255.255", MULTIPLAYER_PORT))
        except OSError as exc:
            print(f"[MP] state sync failed: {exc}")

    while True:
        try:
            data, addr = sock.recvfrom(65535)
        except BlockingIOError:
            break
        except OSError as exc:
            print(f"[MP] receive failed: {exc}")
            break

        try:
            packet = json.loads(data.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            continue

        if packet.get("id") == MULTIPLAYER_ID:
            continue

        if packet.get("kind") == "vASW-action":
            if multiplayer_is_host_role() and packet.get("action") == "launch":
                apply_multiplayer_launch_action(packet)
            elif multiplayer_is_host_role() and packet.get("action") == "contacts":
                apply_multiplayer_contact_contribution(packet)
            elif multiplayer_is_host_role() and packet.get("action") == "contact_command":
                apply_multiplayer_contact_command(packet)
            continue

        if packet.get("kind") == "vASW-host" or packet.get("role") == "HOST":
            host = {
                "id": str(packet.get("id")),
                "callsign": str(packet.get("callsign", "HOST")),
                "aircraft_type": str(packet.get("aircraft_type", "ASW")),
                "password_required": bool(packet.get("password_required", packet.get("role") == "SERVER")),
                "addr": addr[0],
                "last_seen": now
            }
            first_host_seen = multiplayer_host_seen is None or multiplayer_host_seen.get("id") != host["id"]
            multiplayer_host_seen = host
            if first_host_seen and multiplayer_role == "JOIN":
                print(f"[MP] host available: {host['callsign']} from {host['addr']}")
            sync_multiplayer_menu_status()

        if packet.get("kind") not in ("vASW-aircraft", "vASW-peer"):
            if (
                packet.get("kind") == "vASW-state" and
                multiplayer_role == "JOIN" and
                multiplayer_host_seen is not None and
                str(packet.get("id")) == str(multiplayer_host_seen.get("id")) and
                multiplayer_can_join_server()
            ):
                apply_multiplayer_state(packet)
            continue

        try:
            peer = {
                "id": str(packet.get("id")),
                "callsign": str(packet.get("callsign", "MP")),
                "aircraft_type": str(packet.get("aircraft_type", "ASW")),
                "player_type": str(packet.get("player_type", "Aircraft")),
                "team": str(packet.get("team", "BLUFOR")),
                "lat": float(packet["lat"]),
                "lon": float(packet["lon"]),
                "hdg": float(packet.get("hdg", 0)),
                "alt": float(packet.get("alt", 0)),
                "depth": float(packet.get("depth", 0)),
                "speed": float(packet.get("speed", 0)),
                "addr": addr[0],
                "last_seen": now
            }
        except (KeyError, TypeError, ValueError):
            continue

        first_seen = peer["id"] not in multiplayer_peers
        multiplayer_peers[peer["id"]] = peer
        if first_seen:
            print(f"[MP] saw {peer.get('player_type', 'Aircraft').lower()} {peer['callsign']} from {peer['addr']}")
            if multiplayer_is_host_role():
                update_multiplayer_channel_assignments()
                multiplayer_last_state_broadcast = 0.0

    for peer_id, peer in list(multiplayer_peers.items()):
        if now - peer.get("last_seen", 0) > MULTIPLAYER_STALE_SECONDS:
            print(f"[MP] lost aircraft {peer.get('callsign', peer_id)}")
            multiplayer_peers.pop(peer_id, None)
            if multiplayer_is_host_role():
                update_multiplayer_channel_assignments()
                multiplayer_last_state_broadcast = 0.0

    if multiplayer_host_seen and now - multiplayer_host_seen.get("last_seen", 0) > MULTIPLAYER_STALE_SECONDS:
        if multiplayer_role == "JOIN":
            print(f"[MP] lost host {multiplayer_host_seen.get('callsign', 'HOST')}")
        multiplayer_host_seen = None

    if in_menu:
        sync_multiplayer_menu_status()


def draw_remote_aircraft_map(surface):
    if not multiplayer_enabled:
        return

    for peer in multiplayer_peers.values():
        peer_world = pygame.Vector2(latlong_to_pix(peer["lat"], peer["lon"]))
        peer_screen = map_layer.translate_point(peer_world)
        pygame.draw.circle(surface, (255, 190, 80), peer_screen, 9, 1)
        draw_heading_line(surface, peer_screen, peer["hdg"] - 90, 22 * map_layer.zoom, (255, 190, 80), 2)
        label = font.render(multiplayer_aircraft_tag(peer), False, (255, 190, 80))
        surface.blit(label, (peer_screen[0] + 10, peer_screen[1] - 12))


def draw_remote_aircraft_radar(surface, player_world, center, radar_radius, pixels_per_nm, rotation_deg):
    if not multiplayer_enabled:
        return

    for peer in multiplayer_peers.values():
        peer_world = pygame.Vector2(latlong_to_pix(peer["lat"], peer["lon"]))
        peer_screen = radar_point_from_world(peer_world, player_world, center, pixels_per_nm, rotation_deg)
        if center.distance_to(peer_screen) > radar_radius:
            continue
        pygame.draw.circle(surface, (255, 190, 80), peer_screen, 8, 1)
        display_hdg = (peer["hdg"] - rotation_deg) % 360
        pointer_end = (
            peer_screen.x + math.sin(math.radians(display_hdg)) * 20,
            peer_screen.y - math.cos(math.radians(display_hdg)) * 20
        )
        pygame.draw.line(surface, (255, 190, 80), peer_screen, pointer_end, 2)
        label = font.render(multiplayer_aircraft_tag(peer), False, (255, 190, 80))
        surface.blit(label, (peer_screen.x + 9, peer_screen.y - 10))


def radar_point_from_latlon(target_lat, target_lon, own_lat, own_lon, center, pixels_per_nm, rotation_deg):
    distance_nm = haversine(own_lat, own_lon, target_lat, target_lon)
    bearing = haversine_bearing(own_lat, own_lon, target_lat, target_lon)
    display_bearing = (bearing - rotation_deg) % 360

    return pygame.Vector2(
        center[0] + math.sin(math.radians(display_bearing)) * distance_nm * pixels_per_nm,
        center[1] - math.cos(math.radians(display_bearing)) * distance_nm * pixels_per_nm
    )


def line_bounds_near_radar(bounds, own_lat, own_lon, radar_range_nm):
    min_lat, max_lat, min_lon, max_lon = bounds
    lat_margin = radar_range_nm / 60 + 0.25
    lon_scale = max(0.2, math.cos(math.radians(own_lat)))
    lon_margin = radar_range_nm / (60 * lon_scale) + 0.25

    return (
        max_lat >= own_lat - lat_margin and
        min_lat <= own_lat + lat_margin and
        max_lon >= own_lon - lon_margin and
        min_lon <= own_lon + lon_margin
    )


def interpolate_latlon_points(start, end, spacing_nm):
    start_lat, start_lon = start
    end_lat, end_lon = end
    segment_nm = haversine(start_lat, start_lon, end_lat, end_lon)
    steps = max(1, min(24, int(segment_nm / spacing_nm)))

    for step in range(1, steps + 1):
        t = step / steps
        yield (
            start_lat + (end_lat - start_lat) * t,
            start_lon + (end_lon - start_lon) * t
        )


def radar_terrain_cache_key(size, own_lat, own_lon, radar_range_nm, rotation_deg):
    position_precision = 0.01 if radar_range_nm <= 5 else 0.02 if radar_range_nm <= 20 else 0.04 if radar_range_nm <= 60 else 0.08
    heading_precision = 30

    return (
        size,
        radar_range_nm,
        round(own_lat / position_precision),
        round(own_lon / position_precision),
        round(rotation_deg / heading_precision)
    )


def draw_cached_radar_terrain(surface, own_lat, own_lon, center, radar_radius, radar_range_nm, pixels_per_nm, rotation_deg):
    global radar_terrain_cache

    key = radar_terrain_cache_key(surface.get_size(), own_lat, own_lon, radar_range_nm, rotation_deg)
    if radar_terrain_cache["key"] != key or radar_terrain_cache["surface"] is None:
        terrain_surface = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
        draw_radar_coastline(terrain_surface, own_lat, own_lon, center, radar_radius, radar_range_nm, pixels_per_nm, rotation_deg)
        radar_terrain_cache = {
            "key": key,
            "surface": terrain_surface
        }

    surface.blit(radar_terrain_cache["surface"], (0, 0))


def draw_radar_land(surface, own_lat, own_lon, center, radar_radius, radar_range_nm, pixels_per_nm, rotation_deg):
    if land_raster_surface is None:
        return

    raster_width, raster_height = land_raster_surface.get_size()
    lat_margin = (radar_range_nm / 60) * 1.42
    lon_scale = max(0.2, math.cos(math.radians(own_lat)))
    lon_margin = (radar_range_nm / (60 * lon_scale)) * 1.42

    min_lat = max(-90, own_lat - lat_margin)
    max_lat = min(90, own_lat + lat_margin)
    min_lon = max(-180, own_lon - lon_margin)
    max_lon = min(180, own_lon + lon_margin)

    left = max(0, min(raster_width - 1, lon_to_raster_x(min_lon, raster_width)))
    right = max(0, min(raster_width, lon_to_raster_x(max_lon, raster_width)))
    top = max(0, min(raster_height - 1, lat_to_raster_y(max_lat, raster_height)))
    bottom = max(0, min(raster_height, lat_to_raster_y(min_lat, raster_height)))

    if right <= left or bottom <= top:
        return

    crop = land_raster_surface.subsurface(pygame.Rect(left, top, right - left, bottom - top)).copy()
    terrain_size = max(1, int(radar_radius * 2))
    terrain_surface = pygame.transform.scale(crop, (terrain_size, terrain_size))

    if rotation_deg:
        terrain_surface = pygame.transform.rotate(terrain_surface, rotation_deg)

    mask_surface = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
    pygame.draw.circle(mask_surface, (255, 255, 255, 255), center, int(radar_radius))
    terrain_layer = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
    terrain_rect = terrain_surface.get_rect(center=(int(center.x), int(center.y)))
    terrain_layer.blit(terrain_surface, terrain_rect)
    terrain_layer.blit(mask_surface, (0, 0), special_flags=pygame.BLEND_RGBA_MULT)
    surface.blit(terrain_layer, (0, 0))


def draw_radar_coastline(surface, own_lat, own_lon, center, radar_radius, radar_range_nm, pixels_per_nm, rotation_deg):
    if not coastline_polylines:
        return

    coastline_colour = (190, 195, 205)
    max_distance_nm = radar_range_nm * 1.08
    min_pixel_step = 1.2

    for coastline in query_terrain_index(coastline_index, own_lat, own_lon, radar_range_nm):
        if not line_bounds_near_radar(coastline["bounds"], own_lat, own_lon, radar_range_nm):
            continue

        visible_segment = []
        last_point = None

        for lat, lon in coastline["points"]:
            distance_nm = haversine(own_lat, own_lon, lat, lon)
            if distance_nm <= max_distance_nm:
                point = radar_point_from_latlon(lat, lon, own_lat, own_lon, center, pixels_per_nm, rotation_deg)
                if center.distance_to(point) <= radar_radius + 2:
                    if last_point is None or last_point.distance_to(point) >= min_pixel_step:
                        visible_segment.append(point)
                        last_point = point
                    continue

            if len(visible_segment) >= 2:
                pygame.draw.lines(surface, coastline_colour, False, visible_segment, 1)
            visible_segment = []
            last_point = None

        if len(visible_segment) >= 2:
            pygame.draw.lines(surface, coastline_colour, False, visible_segment, 1)


def contact_domain(contact):
    classification_type = getattr(contact, "classification_type", "Unknown")
    if getattr(contact, "internal_type", "") == "Surface-Ship":
        classification_type = "Surface-Ship"
    if classification_type == "Air":
        return "A"
    if classification_type == "Surface-Ship":
        return "SU"
    if classification_type in ("Biological", "Submarine"):
        return "SS"
    if classification_type == "Land":
        return "L"
    return "SS"


def contact_is_civilian_surface(contact):
    return (
        getattr(contact, "internal_type", "") == "Surface-Ship" and
        getattr(contact, "internal_class", "") == "Civilian"
    )


def contact_is_surface_ship(contact):
    return getattr(contact, "internal_type", "") == "Surface-Ship"


def contact_is_radar_visible(contact):
    if is_dicass_ping_contact(contact):
        return False
    if getattr(contact, "broadcasting", False):
        return True
    return bool(getattr(contact, "detected", False))


def radar_contact_tag(contact):
    track_number = getattr(contact, "track_number", "")
    speed = float(getattr(contact, "speed", 0) or 0)
    contact_class = getattr(contact, "classification_class", "Unknown")
    if contact_is_civilian_surface(contact):
        contact_class = "Civilian"

    parts = [str(track_number), f"{speed:.0f}KT"]
    parts.append("UNK" if not contact_class or contact_class == "Unknown" else str(contact_class)[:3].upper())
    return " ".join(parts)


def build_track_quality_matrix():
    columns = ["P", "U", "F", "?F", "N", "S", "H"]
    rows = ["A", "SU", "SS", "L"]
    colours = {row: {column: (0, 0, 0) for column in columns} for row in rows}

    for contact in contacts:
        if is_dicass_ping_contact(contact):
            continue

        row = contact_domain(contact)
        status = getattr(contact, "identity_status", "P")
        if status not in columns:
            status = "P"
        colours[row][status] = contact_identity_colours.get(status, (90, 210, 245))

    return rows, columns, colours


def draw_track_quality_matrix(surface):
    rows, columns, colours = build_track_quality_matrix()
    cell = 20
    label_w = 26
    label_h = 18
    gap = 2
    panel_w = label_w + len(columns) * (cell + gap) + 8
    panel_h = label_h + len(rows) * (cell + gap) + 8
    x0 = surface.get_width() - panel_w - 18
    y0 = surface.get_height() - panel_h - 18
    border_colour = (170, 175, 190)
    text_colour = (230, 230, 235)
    matrix_font = scaled_sys_font(12, bold=True)

    pygame.draw.rect(surface, (18, 36, 134), pygame.Rect(x0, y0, panel_w, panel_h))
    pygame.draw.rect(surface, border_colour, pygame.Rect(x0, y0, panel_w, panel_h), 1)

    for col_index, column in enumerate(columns):
        label = matrix_font.render(column, False, text_colour)
        x = x0 + label_w + col_index * (cell + gap)
        surface.blit(label, (x + (cell - label.get_width()) / 2, y0 + 2))

    for row_index, row in enumerate(rows):
        y = y0 + label_h + row_index * (cell + gap)
        row_label = matrix_font.render(row, False, text_colour)
        surface.blit(row_label, (x0 + 4, y + (cell - row_label.get_height()) / 2))

        for col_index, column in enumerate(columns):
            x = x0 + label_w + col_index * (cell + gap)
            rect = pygame.Rect(x, y, cell, cell)
            pygame.draw.rect(surface, colours[row][column], rect)
            pygame.draw.rect(surface, border_colour, rect, 1)


def draw_radar_trail(surface, trail, ownship_world_pos, center, pixels_per_nm, rotation_deg, colour):
    def translate(point):
        return radar_point_from_world(point, ownship_world_pos, center, pixels_per_nm, rotation_deg)

    draw_fading_trail(surface, trail, translate, colour, 2)


def blit_radar_icon(surface, image, center_pos, size):
    if image.get_bitsize() not in (24, 32):
        image = image.convert_alpha()
    icon = pygame.transform.scale(image, (size, size))
    rect = icon.get_rect(center=(int(center_pos.x), int(center_pos.y)))
    surface.blit(icon, rect)


def tinted_icon(image, colour, size):
    icon = pygame.transform.scale(image.convert_alpha(), (size, size))
    mask = pygame.mask.from_surface(icon)
    return mask.to_surface(setcolor=(*colour, 255), unsetcolor=(0, 0, 0, 0)).convert_alpha()


def blit_contact_icon(surface, contact, center_pos, size):
    icon = tinted_icon(unknown_contact_surface, contact_display_colour(contact), size)
    rect = icon.get_rect(center=(int(center_pos.x), int(center_pos.y)))
    surface.blit(icon, rect)
    return rect


def draw_circle_inside_radar(surface, colour, circle_center, circle_radius, radar_center, radar_radius, width=1):
    clipped_surface = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
    pygame.draw.circle(clipped_surface, colour, circle_center, circle_radius, width)

    radar_mask = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
    pygame.draw.circle(radar_mask, (255, 255, 255, 255), radar_center, int(radar_radius))
    clipped_surface.blit(radar_mask, (0, 0), special_flags=pygame.BLEND_RGBA_MULT)
    surface.blit(clipped_surface, (0, 0))


def draw_radar_bearing_detection(surface, sono_screen, bearing_deg, uncertainty_deg, rotation_deg, radar_radius, colour=(255, 80, 80)):
    centre_bearing = (bearing_deg - rotation_deg) % 360

    def endpoint(display_bearing, length_scale=1.0):
        length = radar_radius * length_scale
        return pygame.Vector2(
            sono_screen.x + math.sin(math.radians(display_bearing)) * length,
            sono_screen.y - math.cos(math.radians(display_bearing)) * length
        )

    if uncertainty_deg <= 1.0:
        pygame.draw.line(surface, colour, sono_screen, endpoint(centre_bearing), 1)
        return

    uncertainty_deg = min(85.0, max(2.0, uncertainty_deg))
    left = endpoint(centre_bearing - uncertainty_deg)
    right = endpoint(centre_bearing + uncertainty_deg)
    centre = endpoint(centre_bearing, 0.88)
    wedge_overlay = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
    pygame.draw.polygon(wedge_overlay, (*colour, 16), [sono_screen, left, right])
    surface.blit(wedge_overlay, (0, 0))
    pygame.draw.line(surface, colour, sono_screen, left, 1)
    pygame.draw.line(surface, colour, sono_screen, right, 1)
    pygame.draw.line(surface, colour, left, right, 1)
    pygame.draw.line(surface, (255, 135, 135), sono_screen, centre, 1)


def draw_radar_display(surface):
    surface.fill((18, 36, 134))

    width, height = surface.get_size()
    center = pygame.Vector2(width / 2, height / 2)
    radar_radius = min(width, height) * 0.43
    radar_range_nm = radar_range_options[radar_range_index]
    pixels_per_nm = radar_radius / radar_range_nm
    rotation_deg = hdg if radar_orientation == "TRACK" else 0
    player_world = pygame.Vector2(latlong_to_pix(player_pos.x, player_pos.y))
    own_lat, own_lon = pix_to_latlong(player_world.x, player_world.y)
    radar_line_colour = (70, 75, 90)
    radar_text_colour = (225, 225, 230)

    draw_cached_radar_terrain(surface, own_lat, own_lon, center, radar_radius, radar_range_nm, pixels_per_nm, rotation_deg)

    for ring in (0.25, 0.5, 0.75, 1.0):
        ring_radius = int(radar_radius * ring)
        pygame.draw.circle(surface, radar_line_colour, center, ring_radius, 1)
        range_label = font.render(f"{radar_range_nm * ring:g}", False, radar_text_colour)
        surface.blit(range_label, (center.x + ring_radius + 4, center.y - 8))

    for bearing_mark in range(0, 360, 30):
        display_bearing = (bearing_mark - rotation_deg) % 360
        outer = pygame.Vector2(
            center.x + math.sin(math.radians(display_bearing)) * radar_radius,
            center.y - math.cos(math.radians(display_bearing)) * radar_radius
        )
        inner = pygame.Vector2(
            center.x + math.sin(math.radians(display_bearing)) * (radar_radius - 10),
            center.y - math.cos(math.radians(display_bearing)) * (radar_radius - 10)
        )
        pygame.draw.line(surface, radar_line_colour, inner, outer, 1)

        label_pos = pygame.Vector2(
            center.x + math.sin(math.radians(display_bearing)) * (radar_radius + 18),
            center.y - math.cos(math.radians(display_bearing)) * (radar_radius + 18)
        )
        bearing_label = font.render(f"{bearing_mark:03d}", False, radar_text_colour)
        surface.blit(
            bearing_label,
            (label_pos.x - bearing_label.get_width() / 2, label_pos.y - bearing_label.get_height() / 2)
        )

    heading_label = "TRACK UP" if radar_orientation == "TRACK" else "NORTH UP"
    mode_label = font.render(f"RADAR {heading_label} RANGE {radar_range_nm:g}NM", False, radar_text_colour)
    surface.blit(mode_label, (12, 46))

    blit_radar_icon(surface, sonoT_surface, center, 28)
    ownship_heading = -90 if radar_orientation == "TRACK" else hdg - 90
    draw_heading_line(surface, center, ownship_heading, 28, (0, 220, 255), 2)
    draw_remote_aircraft_radar(surface, player_world, center, radar_radius, pixels_per_nm, rotation_deg)

    for contact in contacts:
        if not hasattr(contact, "contact_lat") or not hasattr(contact, "contact_long"):
            continue
        if not contact_is_radar_visible(contact):
            continue

        contact_world = pygame.Vector2(latlong_to_pix(contact.contact_lat, contact.contact_long))
        contact_screen = radar_point_from_world(contact_world, player_world, center, pixels_per_nm, rotation_deg)
        if center.distance_to(contact_screen) > radar_radius:
            continue

        contact_colour = contact_display_colour(contact)
        draw_radar_trail(surface, getattr(contact, "trail", []), player_world, center, pixels_per_nm, rotation_deg, contact_colour)
        draw_contact_direction_line(surface, contact_screen, getattr(contact, "bearing", 0), rotation_deg, colour=contact_colour, width=2)
        blit_contact_icon(surface, contact, contact_screen, 22)
        if torpedo_designated_contact is contact:
            pygame.draw.circle(surface, (255, 230, 90), contact_screen, 18, 2)
            target_label = font.render("TGT", False, (255, 230, 90))
            surface.blit(target_label, (contact_screen.x + 8, contact_screen.y + 8))
        track_label = font.render(radar_contact_tag(contact), False, contact_colour)
        surface.blit(track_label, (contact_screen.x + 7, contact_screen.y - 8))

    for sono in sono_array:
        sono_screen = radar_point_from_world(pygame.Vector2(sono.x, sono.y), player_world, center, pixels_per_nm, rotation_deg)
        if center.distance_to(sono_screen) <= radar_radius:
            blit_radar_icon(surface, sonoD_surface, sono_screen, 22)
            channel_label = font.render(f"D{sono.channel}", False, pygame.Color(sono.colour))
            surface.blit(channel_label, (sono_screen.x + 7, sono_screen.y - 9))

            if getattr(sono, "range_circle", False):
                range_ring_radius = int(min(radar_radius, DIFAR_REFERENCE_RANGE_NM * pixels_per_nm))
                draw_circle_inside_radar(surface, (0, 255, 0), sono_screen, range_ring_radius, center, radar_radius, 1)

            if bearing_lines_visible and getattr(sono, "bearing_lines_visible", True):
                for detection in getattr(sono, "detections", []):
                    if getattr(detection.get("contact"), "bearing_lines_hidden", False):
                        continue
                    draw_radar_bearing_detection(
                        surface,
                        sono_screen,
                        detection["bearing"],
                        detection.get("uncert", 0),
                        rotation_deg,
                        radar_radius
                    )

    draw_manual_azigram_bearing_lines_radar(surface, center, radar_radius, rotation_deg)

    for active_sono in active_sono_array:
        active_screen = radar_point_from_world(pygame.Vector2(active_sono.x, active_sono.y), player_world, center, pixels_per_nm, rotation_deg)
        if center.distance_to(active_screen) <= radar_radius:
            blit_radar_icon(surface, sonoS_surface, active_screen, 22)
            channel_label = font.render(f"S{active_sono.channel}", False, radar_text_colour)
            surface.blit(channel_label, (active_screen.x + 7, active_screen.y - 9))

    for xbt_pos, label in iter_xbt_positions():
        xbt_screen = radar_point_from_world(xbt_pos, player_world, center, pixels_per_nm, rotation_deg)
        if center.distance_to(xbt_screen) <= radar_radius:
            blit_radar_icon(surface, sonoB_surface, xbt_screen, 22)
            surface.blit(font.render(str(label), False, (0, 220, 235)), (xbt_screen.x + 7, xbt_screen.y - 9))

    for torpedo in torp_array:
        draw_radar_trail(surface, torpedo.trail, player_world, center, pixels_per_nm, rotation_deg, (120, 210, 255))
        torp_screen = radar_point_from_world(torpedo.pos, player_world, center, pixels_per_nm, rotation_deg)
        if center.distance_to(torp_screen) <= radar_radius:
            blit_radar_icon(surface, torpedo_surface, torp_screen, 24)
            torp_display_heading = getattr(torpedo, "display_heading", torpedo.heading)
            torp_bearing = (torp_display_heading - rotation_deg) % 360
            pointer_end = (
                torp_screen.x + math.sin(math.radians(torp_bearing)) * 18,
                torp_screen.y - math.cos(math.radians(torp_bearing)) * 18
            )
            pygame.draw.line(surface, (255, 230, 120), torp_screen, pointer_end, 2)
            if torpedo.target is not None:
                target_world = pygame.Vector2(latlong_to_pix(torpedo.target.contact_lat, torpedo.target.contact_long))
                target_screen = radar_point_from_world(target_world, player_world, center, pixels_per_nm, rotation_deg)
                if center.distance_to(target_screen) <= radar_radius:
                    draw_dotted_line(surface, (255, 230, 120), torp_screen, target_screen, 1)

    draw_track_quality_matrix(surface)


def draw_ship_routes(surface):
    route_font = scaled_sys_font(11, bold=True)
    for contact in contacts:
        if getattr(contact, "internal_type", "") != "Surface-Ship":
            continue
        waypoints = getattr(contact, "route_waypoints", []) or []
        if len(waypoints) < 2:
            continue

        points = []
        for waypoint in waypoints:
            try:
                points.append(pygame.Vector2(map_layer.translate_point(waypoint)))
            except Exception:
                continue
        if len(points) < 2:
            continue

        active = bool(getattr(contact, "route_active", False))
        colour = (120, 230, 170) if active else (90, 125, 120)
        dim_colour = (45, 80, 75)
        pygame.draw.lines(surface, dim_colour, False, points, 5)
        pygame.draw.lines(surface, colour, False, points, 2)

        route_index = max(0, min(int(getattr(contact, "route_index", 0) or 0), len(points) - 1))
        for index, point in enumerate(points):
            radius = 5 if index == route_index and active else 3
            pygame.draw.circle(surface, colour, point, radius, 1)
            if index == route_index and active:
                pygame.draw.circle(surface, (255, 235, 120), point, radius + 4, 1)
                label = route_font.render(f"RTE {getattr(contact, 'track_number', '?')}", False, (255, 235, 120))
                surface.blit(label, (point.x + 8, point.y - 8))

def draw_contact_map_trails(surface):
    for contact in contacts:
        if not contact_is_radar_visible(contact):
            continue
        if not hasattr(contact, "trail") or len(contact.trail) < 2:
            continue
        draw_fading_trail(surface, contact.trail, map_layer.translate_point, contact_display_colour(contact), 2)


def draw_visible_contacts_on_map(surface):
    surface_rect = surface.get_rect()
    for contact in contacts:
        if not hasattr(contact, "contact_lat") or not hasattr(contact, "contact_long"):
            continue
        if not contact_is_radar_visible(contact):
            continue

        screen_pos = map_layer.translate_point(
            latlong_to_pix(contact.contact_lat, contact.contact_long)
        )
        if not surface_rect.inflate(120, 120).collidepoint(screen_pos):
            continue

        contact.contact_rect = draw_map_contact_marker(
            surface,
            contact,
            screen_pos,
            map_layer.zoom
        )
        marker_scale = map_overlay_symbol_scale()
        contact_colour = contact_display_colour(contact)
        track_label = font.render(radar_contact_tag(contact), False, contact_colour)
        surface.blit(
            track_label,
            (screen_pos[0] + int(8 * marker_scale), screen_pos[1] - int(10 * marker_scale))
        )


def draw_contact_creation_range_circles(surface):
    surface_rect = surface.get_rect().inflate(160, 160)
    circle_colour = (145, 145, 145)
    label_colour = (180, 180, 180)

    for row in contact_define_row_array:
        try:
            center_lat = float(getattr(row, "lat_entered", None))
            center_lon = float(getattr(row, "long_entered", None))
            range_nm = float(getattr(row, "range_entered", 0) or 0)
        except (TypeError, ValueError):
            continue
        if range_nm <= 0:
            continue

        center_world = pygame.Vector2(latlong_to_pix(center_lat, center_lon))
        center_screen = pygame.Vector2(map_layer.translate_point(center_world))
        radius_px = range_nm * pixels_per_nm_at(center_world) * map_layer.zoom
        if radius_px <= 1:
            continue
        if not surface_rect.collidepoint(center_screen):
            continue

        draw_dotted_circle(surface, circle_colour, center_screen, radius_px, 1)
        pygame.draw.circle(surface, circle_colour, center_screen, 3, 1)
        label = font.render(f"{range_nm:g} NM", False, label_colour)
        surface.blit(label, (center_screen.x + 6, center_screen.y + 5))


class Sprite(pygame.sprite.Sprite):
    """
    Simple Sprite class for on-screen things
    
    """
    def __init__(self, surface) -> None:
        pygame.sprite.Sprite.__init__(self)
        self.image = surface
        self.rect = surface.get_rect()

class Bullet(pygame.sprite.Sprite):
    def __init__(self):
        super().__init__() #adding super call to make Bullet a pygame Sprite
        self.image = pygame.Surface([4, 10])
        self.image.fill((255,255,255))
        self.rect = self.image.get_rect()
    def update(self):
        self.rect.y -= 3


def lerp(a: float, b: float, t: float) -> float:
    """Linear interpolate on the scale given by a to b, using t as the point on that scale.
    Examples
    --------
        50 == lerp(0, 100, 0.5)
        4.2 == lerp(1, 5, 0.8)
    """
    return (1 - t) * a + t * b

zoom_timer = 0
zoom_duration = 10
zooming = False

submitted = 0
map_centre_x = 0
map_centre_y = 0


time_elapsed = False 
sono_reached_surface = False




zoom_threshold = [4,10,20]




map_group.center((8000/2,4000/2))

torpedo_surface = pygame.image.load("assets/torpedo.png").convert_alpha()
player_sprite = Sprite(torpedo_surface)
""" 
map_group.add(player_sprite) """
alt = None
if alt is not None:
    sono_timer = Timer(2) # SHOULD BE alt/10
else:
    sono_timer = Timer(1)

#100ft/per sec
def move_submarine_pix(x, y, speed_knots, bearing_deg, dt_seconds=1/60, nm_per_pix=2.6):
    """
    Move submarine in pixels given speed in knots and bearing.
    
    x, y: current pixel coordinates
    speed_knots: submarine speed in knots
    bearing_deg: 0 = north, clockwise
    dt_seconds: timestep in seconds (default 1/60)
    nm_per_pix: map scale (default 2.6 NM per pixel)
    """
    # Convert speed to pixels per frame
    pix_per_frame = (speed_knots / nm_per_pix) * (dt_seconds / 3600.0)
    
    # Compute delta x/y
    dx = math.sin(math.radians(bearing_deg)) * pix_per_frame
    dy = -math.cos(math.radians(bearing_deg)) * pix_per_frame  # negative for screen y
    
    # Update position
    x_new = x + dx
    y_new = y + dy
    return pygame.Vector2(x_new, y_new)
map_centre_x = 3830
map_centre_y = 700




offset = 0

def clamp_camera(vec):


    w, h = map_layer.map_rect.size
    vec.x = max(0, min(vec.x, w))
    vec.y = max(0, min(vec.y, h))
    return 

base_path = getattr(sys, '_MEIPASS', os.path.dirname(os.path.abspath(__file__)))
config_path = os.path.join(base_path, 'config.json')

with open(config_path, 'r') as f:
    config = json.load(f)



for sub_conf in config["submarines"]:  # note plural "submarines"
    contact_acoustic_class = sub_conf.get("class", "Akula")
    internal_type = sub_conf.get("internal_type", "Sub-surface")
    internal_class = sub_conf.get("internal_class", contact_acoustic_class if internal_type == "Sub-surface" else "Unknown")
    if internal_type == "Sub-surface" and (internal_class == "Random" or contact_acoustic_class == "Random"):
        internal_class = resolve_submarine_class_selection("Random")
        contact_acoustic_class = internal_class
    broadcasting = bool(sub_conf.get("broadcasting", internal_type == "Surface-Ship"))
    selected_model_library = normalize_model_library(sub_conf.get("model_library", MODEL_LIBRARY_AUTO))
    selected_model = str(sub_conf.get("model", "Auto") or "Auto")
    contact_team = sub_conf.get("team", "Neutral")
    if contact_team not in CONTACT_TEAM_OPTIONS:
        contact_team = "Neutral"
    if internal_type == "Sub-surface":
        SubClass = sub_classes.get(contact_acoustic_class, AkulaSubmarine)  # get the correct subclass
        submarine = SubClass(
            name=sub_conf["name"],
            contact_lat=sub_conf["latitude"],
            contact_long=sub_conf["longitude"],
            speed=sub_conf["speed"],
            depth=sub_conf["depth"],
            bearing=sub_conf["bearing"]
        )
    elif internal_type == "Surface-Ship" and internal_class == "Civilian":
        submarine = make_civilian_surface_contact(
            name=sub_conf["name"],
            contact_lat=sub_conf["latitude"],
            contact_long=sub_conf["longitude"],
            speed=sub_conf["speed"],
            bearing=sub_conf["bearing"]
        )
        submarine.depth = sub_conf["depth"]
    elif internal_type == "Surface-Ship":
        submarine = Contact(
            name=sub_conf["name"],
            tones=[],
            contact_lat=sub_conf["latitude"],
            contact_long=sub_conf["longitude"],
            speed=sub_conf["speed"],
            depth=sub_conf["depth"],
            bearing=sub_conf["bearing"]
        )
        submarine.internal_type = internal_type
        submarine.internal_class = internal_class
    elif internal_type == "Biological":
        if internal_class == "Whale":
            submarine = make_whale_contact(
                name=sub_conf["name"],
                contact_lat=sub_conf["latitude"],
                contact_long=sub_conf["longitude"],
                speed=sub_conf["speed"],
                bearing=sub_conf["bearing"]
            )
            submarine.depth = sub_conf["depth"]
        else:
            submarine = Contact(
                name=sub_conf["name"],
                tones=[],
                contact_lat=sub_conf["latitude"],
                contact_long=sub_conf["longitude"],
                speed=sub_conf["speed"],
                depth=sub_conf["depth"],
                bearing=sub_conf["bearing"]
            )
    else:
        submarine = Contact(
            name=sub_conf["name"],
            tones=[],
            contact_lat=sub_conf["latitude"],
            contact_long=sub_conf["longitude"],
            speed=sub_conf["speed"],
            depth=sub_conf["depth"],
            bearing=sub_conf["bearing"]
        )
    submarine.internal_type = internal_type
    submarine.internal_class = internal_class
    submarine.model_library = selected_model_library
    if selected_model != "Auto":
        submarine.gaist_model_title = selected_model
    elif internal_type == "Surface-Ship":
        submarine.gaist_model_title = ""
        gaist_model_title_for_contact(submarine)
    if internal_type == "Surface-Ship":
        apply_surface_ship_acoustic_profile(submarine)
    submarine.broadcasting = broadcasting
    submarine.team = contact_team
    route_config = ship_route_config_from_saved_config(sub_conf)
    if route_config is not None and configure_ship_route(submarine, route_config):
        apply_ship_route_elapsed_position(submarine)
    apply_saved_contact_state(submarine, sub_conf)
    contacts.append(submarine)

update_all_contact_shadow_following()

# Now `contacts` contains all submarines from the config
# Example access:




if DEDICATED_HOST_MODE:
    multiplayer_role = "SERVER"
    multiplayer_enabled = True
    multiplayer_contact_password = MULTIPLAYER_CONTACT_PASSWORD
    MULTIPLAYER_CALLSIGN = os.environ.get("VASW_HOST_CALLSIGN", "VASW-SERVER")[:12]
    MULTIPLAYER_PLAYER_TYPE = "Aircraft"
    MULTIPLAYER_TEAM = "Neutral"
    ensure_multiplayer_socket()
    try:
        pygame.display.set_caption("vASW Server")
    except Exception:
        pass
    print(f"[SERVER] Dedicated vASW server running as {MULTIPLAYER_CALLSIGN} on UDP {MULTIPLAYER_PORT}")
    start_server_console_thread()

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
# Order of work each frame:
#   1. refresh aircraft position
#   2. process pygame/pygame_gui events
#   3. update sonar/contact state
#   4. draw map, overlays, spectrograms, and UI
while running:
    process_server_commands()
    maybe_server_contact_autosave()
    if state == STATE_MENU:
        draw_menu()


    

    if display_mode == "MAP":
        map_group.draw(map_surface)
    if lat is not None and long is not None:

        x, y = latlong_to_pix(lat, long)
        player_sprite.rect.x = x
        player_sprite.rect.y = y




    if not player_is_ship_or_sub():
        # Refresh the live aircraft position. X-Plane writes a JSON file, so throttle
        # disk reads to avoid doing file I/O 60 times per second.
        if xplane == 1:
            now = time.time()
            if now - last_xplane_read_time >= xplane_read_interval:
                last_xplane_read_time = now
                try:
                    if os.path.getsize(json_path) > 0:
                        with open(json_path, 'r') as f:
                            last_xplane_data = json.load(f)
                except (OSError, json.JSONDecodeError):
                    # The simulator may be writing the file at this exact moment.
                    # Keep using the last good position instead of stalling the UI.
                    pass

            if last_xplane_data is not None:
                lat = last_xplane_data["latitude"]
                long = last_xplane_data["longitude"]
                alt = last_xplane_data["altitude_ft"]
                hdg = last_xplane_data["heading_true"]
        else:
            if aq:
                sim_lat = aq.get("PLANE_LATITUDE")
                sim_long = aq.get("PLANE_LONGITUDE")
                sim_hdg = aq.get("PLANE_HEADING_DEGREES_TRUE")
                sim_alt = aq.get("INDICATED_ALTITUDE")
                sim_plane_alt = aq.get("PLANE_ALTITUDE")
                sim_groundspeed = aq.get("GROUND_VELOCITY")
                if sim_lat is not None and sim_long is not None:
                    lat = sim_lat
                    long = sim_long
                if sim_alt is not None:
                    alt = sim_alt
                if sim_plane_alt is not None:
                    plane_altitude_ft = sim_plane_alt
                if sim_groundspeed is not None:
                    aircraft_groundspeed_kt = sim_groundspeed
                if sim_hdg is not None:
                    hdg = math.degrees(sim_hdg)











    else:
        update_ship_sub_ownship(dt)

    if not update_check_started and time.time() >= update_check_start_time:
        start_background_update_check()
    maybe_show_update_popup()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.VIDEORESIZE:
            screen_width, screen_height = max(640, event.w), max(360, event.h)
            screen = pygame.display.set_mode((screen_width, screen_height), pygame.RESIZABLE)
            update_display_viewport()
            manager.set_window_resolution((screen_width, screen_height))
            refresh_resolution_dependent_fonts()
            resize_ui(screen_width, screen_height)
            layout_top_mode_buttons(screen_width, screen_height)
            sync_stateful_button_styles()
            radar_terrain_cache["key"] = None
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_c and not point_over_visible_ui(pygame.mouse.get_pos()):
                copy_latlon_at_screen_pos(pygame.mouse.get_pos())


        if event.type == pygame.MOUSEWHEEL:
            mouse_pos = pygame.mouse.get_pos()
            if not scroll_contact_define_container(event.y, mouse_pos) and not point_over_visible_ui(mouse_pos):
                if event.y > 0:
                    map_layer.zoom = min(500, map_layer.zoom * 1.3)
                elif event.y < 0:
                    map_layer.zoom = max(1, map_layer.zoom / 1.3)
                map_group.center((map_centre_x, map_centre_y))

        if event.type == pygame.MOUSEBUTTONDOWN:
            handled_search_pattern_chord = False
            if event.button == 1 and in_menu:
                for row in contact_define_row_array:
                    if not getattr(row, "model_dropdown_has_full_options", True) and row.model_dropdown.rect.collidepoint(event.pos):
                        row.ensure_full_model_dropdown()
                        break
            if event.button == 1 and not in_menu:
                for slot in spectrogram_slot_array:
                    if slot.handle_narrowband_drag_start(event.pos):
                        handled_search_pattern_chord = True
                        break
            if event.button == 3:
                for slot in spectrogram_slot_array:
                    if slot.handle_azigram_right_click(event.pos):
                        handled_search_pattern_chord = True
                        break
            if event.button == 3 and getattr(radar_range_button, "visible", True) and radar_range_button.rect.collidepoint(event.pos):
                step_radar_range(-1)
                handled_search_pattern_chord = True
            if event.button in (1, 3) and not in_menu and not point_over_visible_ui(event.pos):
                pressed_buttons = pygame.mouse.get_pressed(3)
                both_primary_buttons = (
                    (event.button == 1 and pressed_buttons[2]) or
                    (event.button == 3 and pressed_buttons[0])
                )
                if both_primary_buttons:
                    pattern_world = right_display_pos_to_world(internal_mouse_pos(event.pos))
                    if pattern_world is not None:
                        open_search_pattern_panel(pattern_world)
                        measure_dragging = False
                        measure_drag_start_world = None
                        measure_drag_current_world = None
                        mouse_down = False
                        handled_search_pattern_chord = True

            if not handled_search_pattern_chord and event.button == 1 and not in_menu and not point_over_visible_ui(event.pos):
                click_internal = internal_mouse_pos(event.pos)
                if ray_trace_mode != "OFF":
                    click_world_raw = right_display_pos_to_world(click_internal)
                    if click_world_raw is not None and handle_ray_trace_click(click_world_raw):
                        handled_search_pattern_chord = True
                if not handled_search_pattern_chord:
                    click_world = snapped_right_display_pos_to_world(click_internal)
                else:
                    click_world = None
                if click_world is not None:
                    now = time.time()
                    double_click = (
                        measure_last_click_pos is not None and
                        now - measure_last_click_time <= 0.35 and
                        pygame.Vector2(event.pos).distance_to(measure_last_click_pos) <= 8
                    )
                    if double_click:
                        ownship_world = pygame.Vector2(latlong_to_pix(player_pos.x, player_pos.y))
                        measure_locked = {"start": ownship_world, "end": click_world}
                        measure_dragging = False
                        measure_drag_start_world = None
                        measure_drag_current_world = None
                        measure_last_click_time = 0.0
                        measure_last_click_pos = None
                    else:
                        measure_locked = None
                        measure_dragging = True
                        measure_drag_start_world = click_world
                        measure_drag_current_world = click_world
                        measure_last_click_time = now
                        measure_last_click_pos = pygame.Vector2(event.pos)

            if not handled_search_pattern_chord and event.button == 3:
                org_mouse_x, org_mouse_y = pygame.mouse.get_pos()
                clicked_contact = find_contact_at_internal_pos(internal_mouse_pos((org_mouse_x, org_mouse_y)))
                if clicked_contact is not None:
                    set_selected_contact(clicked_contact)
                    mouse_down = False
                elif internal_mouse_pos((org_mouse_x, org_mouse_y)).x > INTERNAL_WIDTH / 2:
                    mouse_down = True
                    map_pos_x = map_centre_x
                    map_pos_y = map_centre_y

            if event.button in (4, 5) and scroll_contact_define_container(1 if event.button == 4 else -1, event.pos):
                pass
            elif event.button == 4:  # scroll up
                map_layer.zoom = min(500,map_layer.zoom*1.3)
                map_group.center((map_centre_x,map_centre_y))

            elif event.button == 5:  # scroll down
                map_layer.zoom = max(1,map_layer.zoom/1.3)
                map_group.center((map_centre_x,map_centre_y))

        if event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:
                for slot in spectrogram_slot_array:
                    if slot.handle_narrowband_drag_end(event.pos):
                        measure_dragging = False
                        measure_drag_start_world = None
                        measure_drag_current_world = None
                        break
            if event.button == 1 and measure_dragging:
                release_world = snapped_right_display_pos_to_world(internal_mouse_pos(event.pos))
                if release_world is not None and measure_drag_start_world is not None:
                    if pygame.Vector2(measure_drag_start_world).distance_to(release_world) > 2:
                        measure_locked = {"start": measure_drag_start_world, "end": release_world}
                measure_dragging = False
                measure_drag_start_world = None
                measure_drag_current_world = None
            if event.button == 3:
                mouse_down = False





        if event.type == pygame.MOUSEMOTION:
            for slot in spectrogram_slot_array:
                slot.handle_narrowband_drag_motion(event.pos)

        if event.type == pygame.MOUSEMOTION and measure_dragging:
            current_world = snapped_right_display_pos_to_world(internal_mouse_pos(event.pos))
            if current_world is not None:
                measure_drag_current_world = current_world

        if event.type == pygame_gui.UI_TEXT_ENTRY_CHANGED:
           for row in contact_define_row_array:
                print(row.contact_name_textbox.get_object_ids())
                if event.ui_element == row.contact_name_textbox:
                    text = row.contact_name_textbox.get_text()
                    row.name_entered = text
                    if row.contact_name_textbox.get_text():
                        row.contact_name_label.set_text(f'<font color="#99C979">Contact Name</font>')
                    else:
                        row.contact_name_label.set_text("Contact Name")

                if event.ui_element == row.sub_lat_textbox:
                    text = row.sub_lat_textbox.get_text()

                    if text == "":
                        row.sub_lat_label.set_text("Latitude")  # neutral
                    elif is_valid_lat(text):
                        row.lat_entered = float(text)
                        row.sub_lat_label.set_text('<font color="#99C979">Latitude</font>')
                    else:
                        row.sub_lat_label.set_text('<font color="#b13b3b">Latitude</font>')





                if event.ui_element == row.sub_long_textbox:
                    text = row.sub_long_textbox.get_text()

                    if text == "":
                        row.sub_long_label.set_text("Longitude")  # neutral
                    elif is_valid_lon(text):
                        row.long_entered = float(text)
                        row.sub_long_label.set_text('<font color="#99C979">Longitude</font>')
                    else:
                        row.sub_long_label.set_text('<font color="#b13b3b">Longitude</font>')



                if event.ui_element == row.sub_range_textbox:
                    text = row.sub_range_textbox.get_text()

                    if text == "":
                        row.sub_range_label.set_text("Range (NM)")  # neutral
                    elif float(text) <= 250:
                        row.range_entered = float(text)
                        row.sub_range_label.set_text('<font color="#99C979">Range (NM)</font>')
                    else:
                        row.sub_range_label.set_text('<font color="#b13b3b">Range (NM)</font>')

                if event.ui_element == row.sub_speed_textbox:
                    text = row.sub_speed_textbox.get_text()

                    if text == "":
                        row.sub_speed_label.set_text("Speed (KTS)")  # neutral
                    elif float(text) <= 50:
                        row.speed_entered = float(text)
                        row.sub_speed_label.set_text('<font color="#99C979">Speed (KTS)</font>')
                    else:
                        row.sub_speed_label.set_text('<font color="#b13b3b">Speed (KTS)</font>')



                if event.ui_element == row.sub_depth_textbox:

                    text = row.sub_depth_textbox.get_text()

                    if text == "":
                        row.sub_depth_label.set_text("Depth (M)")  # neutral
                    elif float(text) <= 10000:
                        row.depth_entered = float(text)
                        row.sub_depth_label.set_text('<font color="#99C979">Depth (M)</font>')
                    else:
                        row.sub_depth_label.set_text('<font color="#b13b3b">Depth (M)</font>')


                if event.ui_element == row.sub_bearing_textbox:
                    text = row.sub_bearing_textbox.get_text()

                    if text == "":
                        row.sub_bearing_label.set_text("Bearing (°)")  # neutral
                    elif 0 <= float(text) <= 360:
                        
                            row.bearing_entered = float(text)
                            row.sub_bearing_label.set_text('<font color="#99C979">Bearing (°)</font>')
                    else:
                        row.sub_bearing_label.set_text('<font color="#b13b3b">Bearing (°)</font>')





        if hasattr(pygame_gui, "UI_TEXT_ENTRY_CHANGED") and event.type == pygame_gui.UI_TEXT_ENTRY_CHANGED:
            if event.ui_element in (nav_heading_entry, nav_speed_entry, nav_depth_entry):
                if event.ui_element == nav_heading_entry:
                    ownship_route_active = False
                    ownship_route_status = "Manual heading"
                    nav_route_status_label.set_text(ownship_route_status)
                apply_nav_command_entries(format_fields=False)

        if event.type == pygame_gui.UI_TEXT_ENTRY_FINISHED:
            for row in contact_define_row_array:

                # CONTACT NAME (no formatting needed)
                if event.ui_element == row.contact_name_textbox:
                    text = row.contact_name_textbox.get_text()
                    row.name_entered = text.strip()

                # LATITUDE
                if event.ui_element == row.sub_lat_textbox:
                    text = row.sub_lat_textbox.get_text()

                    try:
                        value = float(text)
                        if -90 <= value <= 90:
                            row.lat_entered = value
                            row.sub_lat_textbox.set_text(f"{value:.5f}")
                        else:
                            row.sub_lat_textbox.set_text("")
                    except:
                        row.sub_lat_textbox.set_text("")

                # LONGITUDE
                if event.ui_element == row.sub_long_textbox:
                    text = row.sub_long_textbox.get_text()

                    try:
                        value = float(text)
                        if -180 <= value <= 180:
                            row.long_entered = value
                            row.sub_long_textbox.set_text(f"{value:.5f}")
                        else:
                            row.sub_long_textbox.set_text("")
                    except:
                        row.sub_long_textbox.set_text("")

                # RANGE (NM)
                if event.ui_element == row.sub_range_textbox:
                    text = row.sub_range_textbox.get_text()

                    try:
                        value = float(text)
                        if 0 <= value <= 250:
                            row.range_entered = value
                            row.sub_range_textbox.set_text(f"{value:.1f}")
                        else:
                            row.sub_range_textbox.set_text("")
                    except:
                        row.sub_range_textbox.set_text("")

                # SPEED (knots)
                if event.ui_element == row.sub_speed_textbox:
                    text = row.sub_speed_textbox.get_text()

                    try:
                        value = float(text)
                        if 0 <= value <= 50:
                            row.speed_entered = value
                            row.sub_speed_textbox.set_text(f"{int(value)}")
                        else:
                            row.sub_speed_textbox.set_text("")
                    except:
                        row.sub_speed_textbox.set_text("")

                # DEPTH (m)
                if event.ui_element == row.sub_depth_textbox:
                    text = row.sub_depth_textbox.get_text()

                    try:
                        value = float(text)
                        if 0 <= value <= 10000:
                            row.depth_entered = value
                            row.sub_depth_textbox.set_text(f"{int(value)}")
                        else:
                            row.sub_depth_textbox.set_text("")
                    except:
                        row.sub_depth_textbox.set_text("")

                # BEARING (°)
                if event.ui_element == row.sub_bearing_textbox:
                    text = row.sub_bearing_textbox.get_text()

                    try:
                        value = float(text)
                        if 0 <= value <= 360:
                            row.bearing_entered = value
                            row.sub_bearing_textbox.set_text(f"{int(value):03d}")
                        else:
                            row.sub_bearing_textbox.set_text("")
                    except:
                        row.sub_bearing_textbox.set_text("")

                if event.ui_element == row.route_textbox:
                    row.route_text_entered = row.route_textbox.get_text().strip()
                if event.ui_element == row.shadow_target_textbox:
                    row.shadow_target_entered = row.shadow_target_textbox.get_text().strip()

            if event.ui_element in (search_pattern_datum_lat_entry, search_pattern_datum_lon_entry):
                apply_typed_search_datum()
            if event.ui_element in (
                search_pattern_heading_entry,
                search_pattern_length_entry,
                search_pattern_spacing_entry,
                search_pattern_buoy_spacing_entry,
                search_pattern_count_entry
            ):
                generate_search_pattern()
            if event.ui_element in (search_pattern_offset_bearing_entry, search_pattern_offset_range_entry):
                apply_search_offset_from_aircraft()
            if event.ui_element == search_pattern_import_entry:
                import_search_pattern_waypoints()
            if event.ui_element in (nav_heading_entry, nav_speed_entry, nav_depth_entry):
                if event.ui_element == nav_heading_entry:
                    ownship_route_active = False
                    ownship_route_status = "Manual heading"
                    nav_route_status_label.set_text(ownship_route_status)
                apply_nav_command_entries()
            if event.ui_element == ship_heading_entry and selected_contact_is_surface_ship():
                heading_text = ship_heading_entry.get_text().strip()
                requested_heading = float(heading_text) if heading_text else current_aircraft_heading_deg(getattr(selected_contact, "bearing", 0))
                request_ship_heading(selected_contact, requested_heading)
                send_multiplayer_contact_command("ship_heading", selected_contact, heading=requested_heading)
                sync_ship_command_controls(update_heading_text=True)
                update_contact_context_panel()
            if event.ui_element == ship_speed_entry and selected_contact_is_surface_ship():
                speed_text = ship_speed_entry.get_text().strip()
                request_ship_speed(selected_contact, speed_text)
                send_multiplayer_contact_command("ship_speed", selected_contact, speed=speed_text)
                sync_ship_command_controls(update_heading_text=True)
                update_contact_context_panel()
            if event.ui_element == ship_route_speed_entry and selected_contact_is_surface_ship():
                route_speed_text = ship_route_speed_entry.get_text().strip()
                set_ship_route_speed(selected_contact, route_speed_text)
                send_multiplayer_contact_command("ship_route_speed", selected_contact, speed=route_speed_text)
                sync_ship_command_controls(update_heading_text=True)
                update_contact_context_panel()
            if event.ui_element == contact_route_entry:
                if selected_contact_is_friendly_surface_ship():
                    route_text = contact_route_entry.get_text()
                    assign_ship_route_from_text(selected_contact, route_text)
                    send_multiplayer_contact_command("ship_route", selected_contact, route_text=route_text)
                    update_contact_context_panel()
            if event.ui_element == nav_route_entry:
                if selected_contact_is_surface_ship():
                    assign_ship_route_from_text(selected_contact, nav_route_entry.get_text())
                    update_contact_context_panel()
                else:
                    import_ownship_route_from_nav_entry()
            if event.ui_element == multiplayer_password_entry:
                update_multiplayer_settings_from_menu()
                sync_multiplayer_menu_status()
            if event.ui_element == multiplayer_callsign_entry:
                update_multiplayer_settings_from_menu()
                sync_multiplayer_menu_status()
                sync_settings_fields()
            if event.ui_element == settings_callsign_entry:
                MULTIPLAYER_CALLSIGN = sanitize_multiplayer_callsign(settings_callsign_entry.get_text())
                settings_callsign_entry.set_text(MULTIPLAYER_CALLSIGN)
                multiplayer_callsign_entry.set_text(MULTIPLAYER_CALLSIGN)
                sync_multiplayer_menu_status()
                sync_settings_fields()

        if event.type == pygame_gui.UI_DROP_DOWN_MENU_CHANGED:
            if event.ui_element == multiplayer_role_dropdown:
                set_multiplayer_role(dropdown_value(event.text))
                sync_settings_fields()
            if event.ui_element == multiplayer_aircraft_dropdown:
                MULTIPLAYER_AIRCRAFT_TYPE = dropdown_value(event.text)
                update_multiplayer_settings_from_menu()
                sync_multiplayer_menu_status()
                sync_settings_fields()
            if event.ui_element == multiplayer_player_type_dropdown:
                MULTIPLAYER_PLAYER_TYPE = dropdown_value(event.text)
                refresh_control_contact_dropdown("Auto")
                update_multiplayer_platform_selector_visibility()
                update_multiplayer_settings_from_menu()
                sync_multiplayer_menu_status()
                update_nav_control_visibility()
            if event.ui_element == multiplayer_team_dropdown:
                MULTIPLAYER_TEAM = dropdown_value(event.text)
                update_multiplayer_settings_from_menu()
                sync_multiplayer_menu_status()
            if event.ui_element == multiplayer_contact_dropdown:
                ownship_control_contact_key = dropdown_value(event.text)
                ownship_control_contact_track = selected_control_contact_track_from_label(ownship_control_contact_key)
                update_multiplayer_settings_from_menu()
                sync_multiplayer_menu_status()
            if event.ui_element == settings_resolution_dropdown:
                apply_resolution_option(event.text)
            if event.ui_element == settings_sound_dropdown:
                apply_sound_level(event.text)
            if event.ui_element == settings_simulator_dropdown:
                set_simulator_mode(event.text)
                sync_settings_fields()
            if event.ui_element == settings_aircraft_dropdown:
                MULTIPLAYER_AIRCRAFT_TYPE = dropdown_value(event.text)
                multiplayer_aircraft_dropdown.selected_option = event.text
                sync_multiplayer_menu_status()
                sync_settings_fields()
            for row in contact_define_row_array:
                if event.ui_element == row.internal_type_dropdown:
                    row.set_internal_type(dropdown_value(event.text))
                if event.ui_element == row.internal_class_dropdown:
                    selected_class = dropdown_value(event.text)
                    row.internal_class_entered = selected_class
                    row.internal_class_label.set_text('<font color="#99C979">Class</font>')
                    if row.internal_type_entered == "Sub-surface":
                        row.class_entered = selected_class
                    else:
                        row.class_entered = ""
                    row.refresh_model_dropdown("Auto")
                    row.update_route_visibility()
                if event.ui_element == row.team_dropdown:
                    row.set_team(dropdown_value(event.text))
                if event.ui_element == row.model_library_dropdown:
                    row.set_model_library(dropdown_value(event.text))
                if event.ui_element == row.model_dropdown:
                    row.ensure_full_model_dropdown()
                    row.set_model(dropdown_value(event.text))

        if event.type == pygame_gui.UI_BUTTON_PRESSED:
            if event.ui_element == settings_button:
                sync_settings_fields()
                set_settings_panel_visible(not settings_panel_visible)
            if event.ui_element == settings_close_button:
                set_settings_panel_visible(False)
            if event.ui_element == settings_update_button:
                check_and_install_update()
            if event.ui_element == update_popup_update_button:
                check_and_install_update()
            if event.ui_element == update_popup_later_button:
                set_update_popup_visible(False)
            if event.ui_element == duplicate_contact_cancel_button:
                set_duplicate_contact_popup_visible(False)
            if event.ui_element == duplicate_contact_override_button:
                if duplicate_contact_pending_row is not None:
                    define_contact_from_row(duplicate_contact_pending_row, override_duplicate=True)
                set_duplicate_contact_popup_visible(False)
            for row in list(contact_define_row_array):
                if event.ui_element == row.broadcasting_checkbox:
                    row.set_broadcasting(not row.broadcasting_entered)
                if event.ui_element == row.delete_contact_button:
                    contact_name = row_contact_name(row)
                    target_contact = next((contact for contact in contacts if str(getattr(contact, "name", "")) == contact_name), None)
                    if multiplayer_role == "JOIN" and not multiplayer_contact_password_ok():
                        print("Delete ignored: enter the contact password first")
                        continue
                    if multiplayer_role == "JOIN" and target_contact is not None:
                        send_multiplayer_contact_command("delete", target_contact)
                    delete_contact_define_row(row, delete_live=True)
                    continue
                
                if event.ui_element == row.define_contact_button:
                    define_contact_from_row(row)
                    continue

        if event.type == pygame_gui.UI_BUTTON_PRESSED: # menu simulator button
            if event.ui_element == map_mode_button:
                display_mode = "MAP"
                update_nav_control_visibility()
                sync_display_mode_control_visibility()
            if event.ui_element == radar_mode_button:
                display_mode = "RADAR"
                update_nav_control_visibility()
                sync_display_mode_control_visibility()
            if event.ui_element == nav_mode_button:
                display_mode = "NAV"
                update_nav_control_visibility()
                sync_display_mode_control_visibility()
            if event.ui_element == radar_orientation_button:
                radar_orientation = "NORTH" if radar_orientation == "TRACK" else "TRACK"
                radar_orientation_button.set_text("NORTH UP" if radar_orientation == "NORTH" else "TRACK UP")
            if event.ui_element == radar_range_button:
                step_radar_range(1)
            if event.ui_element == bearing_lines_button:
                bearing_lines_visible = not bearing_lines_visible
                sync_bearing_lines_button_style()
            if event.ui_element == ship_inject_button:
                ship_injection_enabled = not ship_injection_enabled
                if not ship_injection_enabled:
                    remove_all_injected_ships()
                    clear_xplane_ship_export()
                sync_ship_inject_button_style()
            if event.ui_element == xbt_tab_button:
                xbt_panel_visible = True
                xbt_panel_selected_label = latest_xbt_label()
                update_xbt_panel_selector()
                contact_context_user_closed = True
                contact_context_panel.hide()
                torpedo_designate_button.hide()
                hide_ship_command_controls()
                contact_delete_button.hide()
                contact_context_close_button.hide()
                contact_type_dropdown.hide()
                contact_class_dropdown.hide()
                contact_status_dropdown.hide()
                contact_country_dropdown.hide()
                xbt_panel_close_button.show()
                xbt_raytrace_button.show()
                xbt_raytrace_clear_button.show()
                if xbt_panel_select_dropdown is not None:
                    xbt_panel_select_dropdown.show()
            if event.ui_element == xbt_panel_close_button:
                xbt_panel_visible = False
                xbt_panel_selected_label = None
                xbt_panel_close_button.hide()
                xbt_raytrace_button.hide()
                xbt_raytrace_clear_button.hide()
                if xbt_panel_select_dropdown is not None:
                    xbt_panel_select_dropdown.hide()
                data_Surface.fill((5, 5, 10), rect=pygame.Rect((8, 542), (930, 430)))
            if event.ui_element == xbt_raytrace_button:
                begin_ray_trace_selection()
            if event.ui_element == xbt_raytrace_clear_button:
                clear_ray_trace()
            if event.ui_element == contact_delete_button:
                if selected_contact is not None:
                    if multiplayer_role == "JOIN":
                        if multiplayer_contact_password_ok():
                            send_multiplayer_contact_command("delete", selected_contact)
                            delete_contact(selected_contact)
                        else:
                            print("Delete ignored: enter the contact password first")
                    else:
                        delete_contact(selected_contact)
                continue
            if event.ui_element == contact_context_close_button:
                contact_context_user_closed = True
                hide_contact_context_controls()
            if event.ui_element == search_pattern_generate_button:
                generate_search_pattern()
            if event.ui_element == search_pattern_auto_buoy_button:
                auto_drop_search_pattern_buoys()
            if event.ui_element == auto_buoy_button:
                auto_drop_search_pattern_buoys()
            if event.ui_element == search_pattern_copy_button:
                copy_search_pattern_waypoints()
            if event.ui_element == search_pattern_import_button:
                import_search_pattern_waypoints()
            if event.ui_element == nav_import_route_button:
                if selected_contact_is_surface_ship():
                    assign_ship_route_from_text(selected_contact, nav_route_entry.get_text())
                    update_contact_context_panel()
                else:
                    import_ownship_route_from_nav_entry()
            if event.ui_element == search_pattern_close_button:
                set_search_pattern_panel_visible(False)
            if event.ui_element == search_pattern_apply_datum_button:
                apply_typed_search_datum()
            if event.ui_element == search_pattern_apply_offset_button:
                apply_search_offset_from_aircraft()
            if event.ui_element == search_pattern_save_reference_button:
                save_search_pattern_reference()
            if event.ui_element == search_pattern_clear_reference_button:
                clear_search_pattern_references()

            if event.ui_element == save_button:
                saved_config = {
                    "description": "vASW config",
                    "xplane": xplane,
                    "submarines": []
                }
                if isinstance(config, dict) and "aishub" in config:
                    saved_config["aishub"] = config["aishub"]

                for row in contact_define_row_array:
                    row.update_from_textboxes()
                    # Only save rows with a valid name and class
                    if row.name_entered and row.internal_type_entered and row.internal_class_entered:
                        acoustic_class = row.internal_class_entered if row.internal_type_entered == "Sub-surface" else "Akula"
                        saved_contact = {
                            "name": row.name_entered,
                            "latitude": row.lat_entered,
                            "longitude": row.long_entered,
                            "range": normalized_range_nm(getattr(row, "range_entered", 0)),
                            "speed": row.speed_entered,
                            "depth": row.depth_entered,
                            "bearing": row.bearing_entered,
                            "class": acoustic_class,
                            "internal_type": row.internal_type_entered,
                            "internal_class": row.internal_class_entered,
                            "broadcasting": row.broadcasting_entered,
                            "team": row.team_entered,
                            "shadow_target": row.shadow_target_entered,
                            "shadow_distance_nm": 5.0,
                            "model_library": row.selected_model_library,
                            "model": row.selected_model
                        }
                        row.route_text_entered = row.route_textbox.get_text().strip()
                        row.shadow_target_entered = row.shadow_target_textbox.get_text().strip()
                        if row.shadow_target_entered:
                            saved_contact["shadow_target"] = row.shadow_target_entered
                            saved_contact["shadow_distance_nm"] = 5.0
                        else:
                            saved_contact.pop("shadow_target", None)
                            saved_contact.pop("shadow_distance_nm", None)
                        if row.is_route_enabled() and row.route_text_entered:
                            saved_contact["route_text"] = row.route_text_entered
                        live_contact = next((contact for contact in contacts if getattr(contact, "name", None) == row.name_entered), None)
                        merge_live_contact_state_into_saved(saved_contact, live_contact)
                        if row.shadow_target_entered:
                            saved_contact["shadow_target"] = row.shadow_target_entered
                            saved_contact["shadow_distance_nm"] = 5.0
                        else:
                            saved_contact.pop("shadow_target", None)
                            saved_contact.pop("shadow_distance_nm", None)
                        route_config = ship_route_config_from_contact(live_contact) if live_contact is not None else None
                        if route_config is not None:
                            saved_contact["route"] = route_config
                        elif row.is_route_enabled() and row.route_text_entered:
                            route_config = ship_route_config_from_saved_config(saved_contact)
                            if route_config is not None:
                                saved_contact["route"] = route_config
                        saved_config["submarines"].append(saved_contact)

                with open("config.json", "w") as f:
                    json.dump(saved_config, f, indent=4)
            if event.ui_element == load_button:

                base_path = getattr(sys, '_MEIPASS', os.path.dirname(os.path.abspath(__file__)))
                config_path = os.path.join(base_path, 'config.json')

                # LOAD FILE
                with open(config_path, "r") as f:
                    config = json.load(f)

                contacts.clear()  # clear old contacts

                # Clear old rows if you want a fresh start
                for row in contact_define_row_array:
                    row.row_panel.kill()
                contact_define_row_array.clear()

                # Create new rows as needed
                for i, sub_conf in enumerate(config["submarines"]):
                    # Rebuild both the simulation contact and its editable menu row.
                    contact_acoustic_class = sub_conf.get("class", "Akula")
                    internal_type = sub_conf.get("internal_type", "Sub-surface")
                    internal_class = sub_conf.get("internal_class", contact_acoustic_class if internal_type == "Sub-surface" else "Unknown")
                    if internal_type == "Sub-surface" and (internal_class == "Random" or contact_acoustic_class == "Random"):
                        internal_class = resolve_submarine_class_selection("Random")
                        contact_acoustic_class = internal_class
                    broadcasting = bool(sub_conf.get("broadcasting", internal_type == "Surface-Ship"))
                    contact_team = sub_conf.get("team", "Neutral")
                    if contact_team not in CONTACT_TEAM_OPTIONS:
                        contact_team = "Neutral"
                    shadow_target = shadow_target_name_from_saved_config(sub_conf)
                    shadow_distance_nm = shadow_distance_from_saved_config(sub_conf)
                    selected_model_library = normalize_model_library(sub_conf.get("model_library", MODEL_LIBRARY_AUTO))
                    selected_model = str(sub_conf.get("model", "Auto") or "Auto")
                    spawn_lat, spawn_lon = random_point_within_range_nm(
                        sub_conf["latitude"],
                        sub_conf["longitude"],
                        normalized_range_nm(sub_conf.get("range", 0))
                    )
                    if internal_type == "Sub-surface":
                        SubClass = sub_classes.get(contact_acoustic_class, AkulaSubmarine)
                        submarine = SubClass(
                            name=sub_conf["name"],
                            contact_lat=spawn_lat,
                            contact_long=spawn_lon,
                            speed=sub_conf["speed"],
                            depth=sub_conf["depth"],
                            bearing=sub_conf["bearing"]
                        )
                    elif internal_type == "Surface-Ship" and internal_class == "Civilian":
                        submarine = make_civilian_surface_contact(
                            name=sub_conf["name"],
                            contact_lat=spawn_lat,
                            contact_long=spawn_lon,
                            speed=sub_conf["speed"],
                            bearing=sub_conf["bearing"]
                        )
                        submarine.depth = sub_conf["depth"]
                    elif internal_type == "Surface-Ship":
                        submarine = Contact(
                            name=sub_conf["name"],
                            tones=[],
                            contact_lat=spawn_lat,
                            contact_long=spawn_lon,
                            speed=sub_conf["speed"],
                            depth=sub_conf["depth"],
                            bearing=sub_conf["bearing"]
                        )
                        submarine.internal_type = internal_type
                        submarine.internal_class = internal_class
                        gaist_model_title_for_contact(submarine)
                        apply_surface_ship_acoustic_profile(submarine)
                    elif internal_type == "Biological":
                        if internal_class == "Whale":
                            submarine = make_whale_contact(
                                name=sub_conf["name"],
                                contact_lat=spawn_lat,
                                contact_long=spawn_lon,
                                speed=sub_conf["speed"],
                                bearing=sub_conf["bearing"]
                            )
                            submarine.depth = sub_conf["depth"]
                        else:
                            submarine = Contact(
                                name=sub_conf["name"],
                                tones=[],
                                contact_lat=spawn_lat,
                                contact_long=spawn_lon,
                                speed=sub_conf["speed"],
                                depth=sub_conf["depth"],
                                bearing=sub_conf["bearing"]
                            )
                    else:
                        submarine = Contact(
                            name=sub_conf["name"],
                            tones=[],
                            contact_lat=spawn_lat,
                            contact_long=spawn_lon,
                            speed=sub_conf["speed"],
                            depth=sub_conf["depth"],
                            bearing=sub_conf["bearing"]
                        )
                    submarine.internal_type = internal_type
                    submarine.internal_class = internal_class
                    submarine.model_library = selected_model_library
                    if selected_model != "Auto":
                        submarine.gaist_model_title = selected_model
                    elif internal_type == "Surface-Ship":
                        submarine.gaist_model_title = ""
                        gaist_model_title_for_contact(submarine)
                        apply_surface_ship_acoustic_profile(submarine)
                    submarine.broadcasting = broadcasting
                    submarine.team = contact_team
                    submarine.shadow_target_name = shadow_target
                    submarine.shadow_distance_nm = shadow_distance_nm
                    route_config = ship_route_config_from_saved_config(sub_conf)
                    if route_config is not None and configure_ship_route(submarine, route_config):
                        apply_ship_route_elapsed_position(submarine)
                    apply_saved_contact_state(submarine, sub_conf)
                    contacts.append(submarine)

                    if contact_define_row_array:
                        new_x = contact_define_row_array[-1].row_panel.rect.right + 10
                    else:
                        new_x = 5

                    new_row = ContactDefineRow(y=0, manager=manager, container=contact_define_container)
                    new_row.row_panel.set_relative_position((new_x, 10))  # position next to last row
                    contact_define_row_array.append(new_row)

                    # Populate the row's textboxes
                    new_row.contact_name_textbox.set_text(sub_conf["name"])
                    new_row.sub_lat_textbox.set_text(str(sub_conf["latitude"]))
                    new_row.sub_long_textbox.set_text(str(sub_conf["longitude"]))
                    new_row.sub_range_textbox.set_text(f"{normalized_range_nm(sub_conf.get('range', 0)):.1f}")
                    new_row.set_internal_type(internal_type, internal_class, selected_model, refresh_model=False)
                    new_row.set_model_library(selected_model_library, selected_model, full_model_options=False)
                    new_row.set_broadcasting(broadcasting)
                    new_row.set_team(contact_team)
                    new_row.shadow_target_textbox.set_text(shadow_target)
                    new_row.shadow_target_entered = shadow_target
                    new_row.shadow_distance_nm_entered = shadow_distance_nm
                    new_row.sub_speed_textbox.set_text(str(int(sub_conf["speed"])))
                    new_row.sub_bearing_textbox.set_text(str(int(sub_conf["bearing"])))
                    new_row.sub_depth_textbox.set_text(str(int(sub_conf["depth"])))
                    new_row.route_textbox.set_text(route_text_from_saved_config(sub_conf))
                    new_row.update_from_textboxes()
                    new_row.update_route_visibility()

                update_all_contact_shadow_following()
                update_contact_define_scroll_area()

            if event.ui_element == start_button:
                update_multiplayer_settings_from_menu()
                if multiplayer_role == "JOIN" and multiplayer_host_seen is None:
                    print("[MP] cannot start as JOIN: no host detected. Start one instance as HOST first.")
                    sync_multiplayer_menu_status()
                    continue
                draw_start_loading_progress(0.08, "Checking multiplayer")
                if multiplayer_is_host_role():
                    draw_start_loading_progress(0.18, "Starting multiplayer server")
                    ensure_multiplayer_socket()
                total_contact_rows = max(1, len(contact_define_row_array))
                for row_index, row in enumerate(contact_define_row_array):
                    draw_start_loading_progress(0.24 + 0.18 * ((row_index + 1) / total_contact_rows), "Preparing contacts")
                    bearing_text = row.sub_bearing_textbox.get_text()
                    if bearing_text:
                        submarine_bearing = float( row.sub_bearing_textbox.get_text())
                    else:
                        submarine_bearing = random.randint(1,360)

                    speed_text =  row.sub_speed_textbox.get_text()
                    if speed_text:
                        submarine_speed = float(speed_text)
                    else:
                        submarine_speed = random.uniform(2.0, 25.0)   
                    class_text = row.internal_class_entered if row.internal_type_entered == "Sub-surface" else ""

                draw_start_loading_progress(0.44, "Resolving mission setup")

                # Resolve class
                if not class_text or str(class_text).lower() == "random":
                    submarine_class = resolve_submarine_class_selection("Random")
                else:
                    submarine_class = class_text

                # Get class data


                state = STATE_GAME
                start_game(draw_start_loading_progress)
                in_menu = False
                set_menu_visible(False)
                draw_start_loading_progress(1.0, "Ready")

            if event.ui_element == civilian_traffic_button:
                generate_dynamic_civilian_traffic()

            if event.ui_element == whale_traffic_button:
                generate_random_whales()
                
            if event.ui_element == simulator_button:
                set_simulator_mode("MSFS" if xplane == 1 else "X-Plane")
                sync_settings_fields()
            


            
            for slot in spectrogram_slot_array:
                if event.ui_element == slot.toggle_range_circle_button:
                    if is_numeric_channel(slot.selected):
                        selected_channel = int(slot.selected)

                        # Find the matching sono
                        matching_sono = None
                        for sono in sono_array:
                            if sono.channel == selected_channel:
                                matching_sono = sono
                                break

                        if matching_sono is not None:
                            # Toggle the range circle
                            matching_sono.range_circle = not matching_sono.range_circle

                            sync_slot_range_circle_button_style(slot)
                if event.ui_element == slot.toggle_bearing_lines_button:
                    if slot.selected is not None:
                        matching_sono = slot.get_passive_sonobuoy_for_slot()
                        if matching_sono is not None:
                            matching_sono.bearing_lines_visible = not getattr(matching_sono, "bearing_lines_visible", True)
                            slot.bearing_lines_visible = matching_sono.bearing_lines_visible
                        else:
                            slot.bearing_lines_visible = not slot.bearing_lines_visible
                    else:
                        slot.bearing_lines_visible = not slot.bearing_lines_visible
                    slot.sync_bearing_lines_button_style()
                if event.ui_element == slot.difar_display_button:
                    slot.display_mode = "AZIGRAM" if slot.display_mode == "SPECTROGRAM" else "SPECTROGRAM"
                    slot.sync_difar_display_button_style()
                if event.ui_element == slot.band_mode_button:
                    slot.band_mode = "NARROWBAND" if slot.band_mode == "BROADBAND" else "BROADBAND"
                    slot.sync_band_mode_button_style()
                if event.ui_element == slot.listen_button:
                    with listen_audio_lock:
                        if listening_spectrogram_slot is slot:
                            listening_spectrogram_slot = None
                        elif slot.get_passive_sonobuoy_for_slot() is not None:
                            listening_spectrogram_slot = slot
                    for display_slot in spectrogram_slot_array:
                        display_slot.sync_listen_button_style()
                if event.ui_element == slot.scuttle_button:
                    if slot.selected != None:
                        with listen_audio_lock:
                            if listening_spectrogram_slot is slot:
                                listening_spectrogram_slot = None
                        if not is_numeric_channel(slot.selected):
                            selected_xbt_label = str(slot.selected)
                            removed_profile = xbt_profiles.pop(selected_xbt_label, None)
                            if selected_xbt_label in sono_channel_array:
                                sono_channel_array.remove(selected_xbt_label)
                            if removed_profile is not None and getattr(removed_profile, "position", None) is not None:
                                xbt_array = [
                                    item for item in xbt_array
                                    if not (
                                        isinstance(item, pygame.Vector2) and item.distance_to(removed_profile.position) < 1
                                    ) and not (
                                        isinstance(item, (tuple, list)) and len(item) >= 2 and
                                        pygame.Vector2(item[0], item[1]).distance_to(removed_profile.position) < 1
                                    )
                                ]
                            if not xbt_profiles:
                                xbt_exists = False
                                latest_xbt_profile = None
                                xbt_panel_selected_label = None
                            elif xbt_panel_selected_label == selected_xbt_label:
                                xbt_panel_selected_label = latest_xbt_label()
                            update_xbt_panel_selector()
                            slot.selected = None
                            for display_slot in spectrogram_slot_array:
                                if display_slot.selected == selected_xbt_label:
                                    display_slot.selected = None
                                display_slot.update_ui()
                                display_slot.sync_listen_button_style()
                            continue

                        channel_number_to_return = int(slot.selected)
                        


                        #if str(channel_number_to_return) in channel_names:
                        channel_names.append(str(channel_number_to_return))
                        if str(channel_number_to_return) in sono_channel_array:
                            sono_channel_array.remove(str(channel_number_to_return))
                        channel_names.sort(key=int)

                        
                        sono_array = [s for s in sono_array if s.channel != channel_number_to_return]
                        active_sono_array = [s for s in active_sono_array if int(s.channel) != channel_number_to_return]
                        spectro_array = [ui for ui in spectro_array if ui.sono.channel != channel_number_to_return]

                        slot.selected = None
    
                        starting_option = "None"

                    
                        channel_names.sort(key=int)
                        sono_channel_array.sort(key=lambda x: int(x) if x.isdigit() else -1)
                        for display_slot in spectrogram_slot_array:
                            if display_slot.selected == channel_number_to_return:
                                display_slot.selected = None
                            display_slot.update_ui()
                            display_slot.sync_listen_button_style()




            if event.ui_element == launch_button:
                if sono_selection != None:
                    if multiplayer_role == "JOIN":
                        if send_multiplayer_launch_request():
                            if sono_selection in ("SSQ-53D(DIFAR)", "SSQ-62(DICASS)", "SSQ-36B(XBT)"):
                                advance_displayed_channel_after_launch(displayed_channel)
                            sonoArmed = False
                            sync_arm_button_style()
                        else:
                            print("[MP] launch blocked: JOIN has no host link")
                        continue
                    
                    if sono_selection == "SSQ-53D(DIFAR)":
                        pending_sono_launch_channel = displayed_channel
                        launch_channel = displayed_channel
                        active_sonobuoy = False
                        launch_sonobuoy()
                        sonoArmed = False
                        sync_arm_button_style()
                        sono_timer_start = pygame.time.get_ticks()
                        
                        


                    elif sono_selection == "SSQ-36B(XBT)":
                        launch_xbt()
                        sonoArmed = False
                        sync_arm_button_style()
                    elif sono_selection == "STINGRAY(TORPEDO)":
                        launch_torp()
                        sonoArmed = False
                        sync_arm_button_style()
                    elif sono_selection == "SSQ-62(DICASS)":
                        pending_sono_launch_channel = displayed_channel
                        launch_channel = displayed_channel
                        launch_active_sonobuoy()
                        sonoArmed = False
                        sync_arm_button_style()
                        sono_timer_start = pygame.time.get_ticks()
            



            if event.ui_element == arm_button:
                arm_sonobuoy()
            if event.ui_element == torpedo_designate_button and selected_contact is not None:
                if torpedo_designated_contact is selected_contact:
                    torpedo_designated_contact = None
                else:
                    torpedo_designated_contact = selected_contact
                sync_torpedo_designate_button_style()
                update_contact_context_panel()
            if event.ui_element == contact_lines_button and selected_contact is not None:
                selected_contact.bearing_lines_hidden = not getattr(selected_contact, "bearing_lines_hidden", False)
                send_multiplayer_contact_command("lines", selected_contact, bearing_lines_hidden=selected_contact.bearing_lines_hidden)
                sync_contact_lines_button_style()
                update_contact_context_panel()
            if event.ui_element == ship_stop_button and selected_contact_is_surface_ship():
                request_ship_stop(selected_contact)
                send_multiplayer_contact_command("ship_stop", selected_contact)
                update_contact_context_panel()
            if event.ui_element == ship_heading_button and selected_contact_is_surface_ship():
                heading_text = ship_heading_entry.get_text().strip()
                requested_heading = float(heading_text) if heading_text else current_aircraft_heading_deg(getattr(selected_contact, "bearing", 0))
                request_ship_heading(selected_contact, requested_heading)
                send_multiplayer_contact_command("ship_heading", selected_contact, heading=requested_heading)
                sync_ship_command_controls(update_heading_text=True)
                update_contact_context_panel()
            if event.ui_element == ship_speed_button and selected_contact_is_surface_ship():
                speed_text = ship_speed_entry.get_text().strip()
                request_ship_speed(selected_contact, speed_text)
                send_multiplayer_contact_command("ship_speed", selected_contact, speed=speed_text)
                sync_ship_command_controls(update_heading_text=True)
                update_contact_context_panel()
            if event.ui_element == ship_resume_route_button and selected_contact_is_surface_ship():
                resume_ship_route(selected_contact)
                send_multiplayer_contact_command("ship_resume_route", selected_contact)
                sync_ship_command_controls(update_heading_text=True)
                update_contact_context_panel()
            if event.ui_element == ship_route_speed_button and selected_contact_is_surface_ship():
                route_speed_text = ship_route_speed_entry.get_text().strip()
                set_ship_route_speed(selected_contact, route_speed_text)
                send_multiplayer_contact_command("ship_route_speed", selected_contact, speed=route_speed_text)
                sync_ship_command_controls(update_heading_text=True)
                update_contact_context_panel()
            if event.ui_element == contact_route_button:
                if selected_contact_is_friendly_surface_ship():
                    route_text = contact_route_entry.get_text()
                    assign_ship_route_from_text(selected_contact, route_text)
                    send_multiplayer_contact_command("ship_route", selected_contact, route_text=route_text)
                    update_contact_context_panel()
                else:
                    print("Route ignored: select a friendly surface ship contact first")
            if event.ui_element == ship_spawn_aft_button:
                if selected_contact_is_surface_ship():
                    spawn_aircraft_aft_of_ship(selected_contact)
                    update_contact_context_panel()
                else:
                    print("Ship aft spawn ignored: select a surface ship contact first")
            if event.ui_element == ship_deck_lock_button:
                if selected_contact_is_surface_ship():
                    toggle_ship_deck_lock(selected_contact)
                    send_multiplayer_contact_command("deck_lock", selected_contact)
                    sync_ship_command_controls(update_heading_text=True)
                    update_contact_context_panel()
                else:
                    print("Deck lock ignored: select a surface ship contact first")
        if event.type == pygame_gui.UI_DROP_DOWN_MENU_CHANGED:


            if not in_menu:
                if event.ui_element == search_pattern_type_dropdown:
                    search_pattern_selected_type = event.text
                    generate_search_pattern()
                if event.ui_element == search_pattern_anchor_mode_dropdown:
                    search_pattern_anchor_mode = event.text
                    generate_search_pattern()
                if xbt_panel_select_dropdown is not None and event.ui_element == xbt_panel_select_dropdown:
                    xbt_panel_selected_label = event.text if event.text in xbt_profiles else None
                if event.ui_element == depth_dropdown:       
                    depth_str = event.text
                    depth = int(depth_str.replace("FT", ""))  # e.g. 400
                if event.ui_element == sonobuoy_dropdown:
                    sono_selection = event.text
                    sync_torpedo_control_visibility()
                if event.ui_element == torpedo_mode_dropdown:
                    selected_torpedo_mode = event.text
                    sync_torpedo_control_visibility()
                if event.ui_element == torpedo_frequency_dropdown:
                    selected_torpedo_frequency = float(event.text)

                if event.ui_element == contact_type_dropdown and selected_contact is not None:
                    selected_contact.operator_classified = True
                    selected_contact.classification_type = event.text
                    class_options = contact_possible_type_list.get(event.text, contact_possible_type_list["Unknown"])
                    selected_contact.classification_class = class_options[0]
                    send_multiplayer_contact_command("classify", selected_contact, classification_type=selected_contact.classification_type, classification_class=selected_contact.classification_class)

                    contact_class_dropdown.kill()
                    contact_class_dropdown = pygame_gui.elements.UIDropDownMenu(
                        options_list=class_options,
                        starting_option=class_options[0],
                        relative_rect=contact_context_dropdown_rect("class"),
                        manager=manager,
                        expansion_height_limit=180
                    )
                    contact_class_dropdown.show()
                    update_contact_context_panel()

                if event.ui_element == contact_class_dropdown and selected_contact is not None:
                    selected_contact.operator_classified = True
                    selected_contact.classification_class = event.text
                    send_multiplayer_contact_command("classify", selected_contact, classification_type=selected_contact.classification_type, classification_class=selected_contact.classification_class)
                    update_contact_context_panel()

                if event.ui_element == contact_status_dropdown and selected_contact is not None:
                    selected_contact.operator_classified = True
                    selected_contact.identity_status = event.text
                    send_multiplayer_contact_command("identity", selected_contact, identity_status=selected_contact.identity_status)
                    update_contact_context_panel()

                if event.ui_element == contact_country_dropdown and selected_contact is not None:
                    selected_contact.operator_classified = True
                    selected_contact.country = event.text
                    send_multiplayer_contact_command("country", selected_contact, country=selected_contact.country)
                    update_contact_context_panel()
                
                for slot in spectrogram_slot_array:
                    if event.ui_element == slot.channel_dropdown:
                        if event.text == "None":
                            slot.selected = None
                        elif is_numeric_channel(event.text):
                            slot.selected = int(event.text)
                        else:
                            slot.selected = None
                        selected_sono = slot.get_passive_sonobuoy_for_slot()
                        if selected_sono is not None:
                            slot.bearing_lines_visible = getattr(selected_sono, "bearing_lines_visible", True)
                        slot.sync_bearing_lines_button_style()
                    if event.ui_element == slot.marker_type_dropdown:
                        slot.marker_type = event.text
                        marker_options = spectrogram_marker_class_list.get(slot.marker_type, ["None"])
                        slot.marker_class = marker_options[0]
                        slot.update_ui()
                        slot.disable_slot_ui()
                    if event.ui_element == slot.marker_class_dropdown:
                        slot.marker_class = event.text

            for contact in contacts:

                if event.ui_element == contact.type_dropdown:

                    selected_type = event.text
                    options = contact_possible_type_list[selected_type]

                    rect = contact.class_dropdown.relative_rect

                    contact.class_dropdown.kill()

                    contact.class_dropdown = pygame_gui.elements.UIDropDownMenu(
                        options,
                        options[0],
                        rect,
                        manager
                    )



                        


                

        

                
            
        manager.process_events(event)
        
    # fill the screen with a color to wipe away anything from last frame




    map_surf.fill((0,0,50))
    internal_surface.fill((30, 30, 30))
    current_time = pygame.time.get_ticks() / 1000  # convert ms to seconds
  

    


 
    keys = pygame.key.get_pressed()
    if lat is None or long is None:
        pass
    else:
        player_pos = pygame.Vector2(float(lat),float(long))
        update_auto_buoy_system()
        update_permanent_ownship_sonar()
        update_manual_deck_lock()
        

    update_multiplayer()
    update_host_civilian_traffic()



    if mouse_down:

        mouse_x, mouse_y = pygame.mouse.get_pos()
        
        dx = org_mouse_x - mouse_x
        dy = org_mouse_y - mouse_y

        # Scale movement by zoom
        map_centre_x = map_pos_x + dx / map_layer.zoom
        map_centre_y = map_pos_y + dy / map_layer.zoom

        map_group.center((map_centre_x, map_centre_y))
                
        


    if sono_timer.check():
        sono_reached_surface = True

    if timer.check():
        time_elapsed = True

    if sono_reached_surface == True:
        if active_sonobuoy == True:
            sono_channel = pending_sono_launch_channel if pending_sono_launch_channel is not None else displayed_channel
            active_world_pos = latlong_to_pix(player_pos.x,player_pos.y)
            spawn_splash(active_world_pos)
            spawn_msfs_splash(player_pos.x, player_pos.y)
            active_sono = Active_Sonobuoy(active_world_pos,depth, sonoS_surface,len(sono_array),sono_channel,2,3,7,200)
            print(displayed_channel, sono_channel)
            active_sono_array.append(active_sono)

            sonoArmed = False
            sync_arm_button_style()
            sono_reached_surface = False
            if str(sono_channel) not in sono_channel_array:
                sono_channel_array.append(str(sono_channel))
            sono_channel_array.sort(key=lambda x: int(x) if x.isdigit() else -1)
            for slot in spectrogram_slot_array:
                slot.update_ui()
            advance_displayed_channel_after_launch(sono_channel)
            pending_sono_launch_channel = None
        else:
            sono_channel = pending_sono_launch_channel if pending_sono_launch_channel is not None else displayed_channel
            sono_world_pos = latlong_to_pix(player_pos.x,player_pos.y)
            spawn_splash(sono_world_pos)
            spawn_msfs_splash(player_pos.x, player_pos.y)
            sono = Sonobuoy(sono_world_pos,depth, sonoD_surface,len(sono_array),sono_channel)
            spectrogram_ui = SpectrogramUI(sono)
            spectro_array.append(spectrogram_ui)
            sono_array.append(sono)



            #sono_channel_array.append(str(len(sono_array)))
            if str(sono_channel) not in sono_channel_array:
                sono_channel_array.append(str(sono_channel))
            sono_channel_array.sort(key=lambda x: int(x) if x.isdigit() else -1)
            sonoArmed = False
            sync_arm_button_style()
            for slot in spectrogram_slot_array:
                slot.update_ui()
            sono_reached_surface = False
            advance_displayed_channel_after_launch(sono_channel)
            pending_sono_launch_channel = None



    for slot in spectrogram_slot_array:
        slot.disable_slot_ui()
    map_overlay_surface.fill((0,0,0,0))

    for contact in contacts:
        if not hasattr(contact, "pos") or not hasattr(contact, "tones"):
            continue
        if any(tone.label == "FMCW" for tone in contact.tones):
            continue
        if not update_contact_shadow_following(contact, dt):
            update_ship_route_following(contact)
            apply_ship_command_dynamics(contact, dt)
            if getattr(contact, "speed", 0) != 0:
                contact.move(dt)
        append_fading_trail(contact, contact.pos, max_points=120)

    update_submarine_reactions(dt)

    for torpedo in torp_array:
        torpedo.update(dt, contacts, draw_map=(display_mode == "MAP"))
    torp_array = [torpedo for torpedo in torp_array if not torpedo.finished]
    update_ship_injections()
    update_msfs_splashes()
    if display_mode == "MAP":
        draw_ship_routes(map_overlay_surface)
        draw_contact_map_trails(map_overlay_surface)
        draw_splash_effects(map_overlay_surface)

    # DICASS pings are represented as temporary synthetic contacts for passive
    # audio. Clear old ping contacts once, then append the current pings below.
    contacts = [
        c for c in contacts
        if not any(tone.label == "FMCW" for tone in c.tones)
    ]

    for i,active_sono in enumerate(active_sono_array):

        active_sono_latlong = pix_to_latlong(active_sono.x,active_sono.y)
        if display_mode == "MAP":
            active_sono.update()

        active_range = 20
        transmitting, current_freq_khz, source_db, progress = active_sono.generate_active_sonar_ping()
        active_sono.current_transmitting = transmitting
        active_sono.current_freq_hz = current_freq_khz
        active_sono.current_source_db = source_db
        active_sono.current_ping_progress = progress
        beat_khz = active_sono.beat_frequency(target_range_nm=active_range, relative_velocity_m_s=2)

        if transmitting:
            active_sono_contact = Contact(
                "DICASS",
                [
                    Tone(
                        freq=current_freq_khz,  # kHz → Hz
                        db=source_db,
                        label="FMCW"
                    )
                ],
                active_sono_latlong[0],
                active_sono_latlong[1],0,400,360
            )

            # Append the new active sonobuoy contact

        else:
            active_sono_contact = Contact(
                "DICASS",
                [
                    Tone(
                        freq=current_freq_khz,  # kHz → Hz
                        db=0,
                        label="FMCW"
                    )
                ],
                active_sono_latlong[0],
                active_sono_latlong[1],0,400,360
            )
        contacts.append(active_sono_contact)

        for contact in contacts:
            if is_dicass_ping_contact(contact):
                continue
            contact_latlong = contact.contact_lat,contact.contact_long
            dist = haversine(contact_latlong[0],contact_latlong[1],pix_to_latlong(active_sono.x,active_sono.y)[0],pix_to_latlong(active_sono.x,active_sono.y)[1])
            contact_depth = float(getattr(contact, "depth", 0) or 0)
            active_depth = float(getattr(active_sono, "depth", 0) or 0)
            two_way_loss = 2.0 * difar_transmission_loss(dist, current_freq_khz)
            layer_loss = environmental_acoustic_loss(dist, current_freq_khz, contact_depth, active_depth)
            target_strength_db = 18.0 if contact_is_submarine(contact) else 10.0
            echo_db = source_db + target_strength_db - two_way_loss - layer_loss
            clutter_db = bottom_clutter_db(dist, active_depth, current_freq_khz)
            contact.active_echo_db = echo_db
            contact.active_clutter_db = clutter_db
            if transmitting and dist < 5 and echo_db >= clutter_db + 3.0:
                render_x,render_y = map_layer.translate_point(latlong_to_pix(contact_latlong[0],contact_latlong[1]))
                mark_contact_detected_unknown(contact)
                if display_mode == "MAP":
                    contact.contact_rect = draw_map_contact_marker(
                        map_overlay_surface,
                        contact,
                        (render_x, render_y),
                        map_layer.zoom
                    )
                contact.active_detected_until = time.time() + 1.0
                


    for i,sono in enumerate(sono_array):
        if channel_is_selected_or_listened(sono.channel):
            sono_lat, sono_lon = pix_to_latlong(sono.x, sono.y)
            for contact in contacts:
                if is_dicass_ping_contact(contact) or not getattr(contact, "tones", None):
                    continue
                dist = haversine(contact.contact_lat, contact.contact_long, sono_lat, sono_lon)
                for tone in contact.tones:
                    received_db = difar_received_db(
                        tone.db,
                        dist,
                        tone.freq,
                        getattr(contact, "depth", 0),
                        getattr(sono, "depth", 0)
                    )
                    received_snr = received_db - difar_background_noise_db(tone.freq)
                    if received_snr < DIFAR_MIN_SNR_DB:
                        tone.received_db.pop(str(sono.channel), None)
                        tone.received_freq.pop(str(sono.channel), None)
                        continue
                    received_freq = doppler_shifted_frequency(tone.freq, contact, sono_lat, sono_lon)
                    tone.set_received_db_and_freq(sono.channel, received_db, received_freq)

        sono.update(offset, draw_map=(display_mode == "MAP"))
      

    player_pos_pix = latlong_to_pix(player_pos.x, player_pos.y)
    screen_pos = map_layer.translate_point((player_pos_pix[0], player_pos_pix[1]))
    if display_mode == "MAP":
        draw_heading_line(map_overlay_surface, (screen_pos), hdg-90, 10*map_layer.zoom, (0, 200, 255), 2)

        player_rect = sonoT_surface.get_rect(center=screen_pos)
        map_overlay_surface.blit(sonoT_surface, player_rect)
        draw_xbt_map_icons(map_overlay_surface)
        draw_manual_azigram_bearing_lines_map(map_overlay_surface)
        draw_remote_aircraft_map(map_overlay_surface)


    for contact in contacts:
        if hasattr(contact, "detecting_buoys"):
            contact.detecting_buoys.clear()
        active_detected_until = getattr(contact, "active_detected_until", 0)
        if (
            not contact_is_surface_ship(contact) and
            not is_dicass_ping_contact(contact) and
            time.time() >= active_detected_until
        ):
            contact.detected = False

    detections_by_contact = {}
    for buoy in sono_array:
        for detection in getattr(buoy, "detections", []):
            if not detection.get("contact_like", True):
                continue
            if detection["uncert"] > DIFAR_LOCALIZATION_MAX_UNCERT_DEG:
                continue
            detections_by_contact.setdefault(detection["contact"], []).append((buoy, detection))

    for contact, buoy_detections in detections_by_contact.items():
        if len(buoy_detections) < 2:
            continue

        has_crossing_bearings = False
        for i in range(len(buoy_detections)):
            for j in range(i + 1, len(buoy_detections)):
                bearing_a = buoy_detections[i][1]["bearing"]
                bearing_b = buoy_detections[j][1]["bearing"]
                bearing_diff = abs((bearing_a - bearing_b + 180) % 360 - 180)
                if bearing_diff > 7.5:
                    has_crossing_bearings = True
                    break
            if has_crossing_bearings:
                break

        if not has_crossing_bearings:
            continue

        for buoy, _ in buoy_detections:
            contact.detecting_buoys.add(buoy)

        render_x, render_y = map_layer.translate_point(
            latlong_to_pix(contact.contact_lat, contact.contact_long)
        )
        mark_contact_detected_unknown(contact)
        if display_mode == "MAP":
            contact.contact_rect = draw_map_contact_marker(
                map_overlay_surface,
                contact,
                (render_x, render_y),
                map_layer.zoom
            )
    if display_mode == "MAP":
        draw_visible_contacts_on_map(map_overlay_surface)
    current_time_sec = time.time()

    if current_time_sec - last_save_time >= SPECTROGRAM_UPDATE_INTERVAL_SEC:
        last_save_time = current_time_sec

        active_spectrogram_channels = set()
        for slot in spectrogram_slot_array:
            if is_numeric_channel(slot.selected):
                active_spectrogram_channels.add(int(slot.selected))
        with listen_audio_lock:
            listening_slot = listening_spectrogram_slot
        if listening_slot is not None and is_numeric_channel(listening_slot.selected):
            active_spectrogram_channels.add(int(listening_slot.selected))

        for spectrogram_ui in spectro_array:
            if spectrogram_ui.sono.channel not in active_spectrogram_channels:
                continue
            spectrogram_ui.update()
    update_spectrogram_listener()
    #draw_contact_menu()
    if display_mode == "MAP":
        draw_contact_creation_range_circles(map_overlay_surface)
        draw_scale_bar(map_overlay_surface, map_layer)
    if not in_menu:
        update_contact_context_panel()
    
    for slot in spectrogram_slot_array:
        slot.draw()
    draw_xbt_panel(data_Surface)
    
    if display_mode == "RADAR":
        draw_radar_display(map_surf)
    elif display_mode == "NAV":
        draw_nav_display(map_surf)
    else:
        map_surf.blit(map_surface,(0,0))
    draw_search_pattern(map_surf)
    draw_ray_trace_overlay(map_surf)
    draw_measurements(map_surf)

    internal_surface.blit(map_surf,(1920/2,0))
    if display_mode == "MAP":
        internal_surface.blit(map_overlay_surface,(1920/2,0))
    internal_surface.blit(data_Surface,(0,0))
    if in_menu:
        internal_surface.blit(menu_surface, (0,0))


    screen.fill((0, 0, 0))
    scaled_surface = pygame.transform.smoothscale(internal_surface, display_viewport_rect.size)
    screen.blit(scaled_surface, display_viewport_rect.topleft)

 #selected_sonobuoy.hz,selected_sonobuoy.db
    
    if in_menu:
        draw_menu()




    dt = clock.tick(60) / 1000



    manager.update(dt)

    manager.draw_ui(screen)
    if not in_menu and any(slot.display_mode == "AZIGRAM" for slot in spectrogram_slot_array):
        key_center = (
            display_viewport_rect.x + int(900 * display_scale),
            display_viewport_rect.y + int(1038 * display_scale)
        )
        key_radius = max(8, int(14 * display_scale))
        draw_azigram_colour_key(screen, key_center, key_radius)

    pygame.display.update()
 
stop_listen_audio()
close_multiplayer_socket()
pygame.quit()




import os

from pathlib import Path

WEB_ADDRESS = '0.0.0.0'
WEB_PORT = '5000'
PROJECT_ROOT = Path(__file__).parent
# TEMPLATES = PROJECT_ROOT / 'droneapp/templates'
STATIC_FOLDER = PROJECT_ROOT / 'droneapp/static'
DEBUG = False
LOG_FILE = 'pytello.log'

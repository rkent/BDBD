# Include 'from .context import bdbd' to import module bdbd into files in this directory
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import bdbd

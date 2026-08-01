#!/usr/bin/env python3
"""Tiny static file server for this folder (browsers won't fetch() local
JSON over file://, so this is needed for the visualizer page)."""
import http.server
import functools
import sys

port = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=str(__import__('pathlib').Path(__file__).parent))
http.server.ThreadingHTTPServer(('127.0.0.1', port), handler).serve_forever()

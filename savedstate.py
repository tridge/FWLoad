#!/usr/bin/env python

import os, json

STATE_FILE = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'jigstate.json')

state_cache = dict()

def mkstate():
    proto = dict()
    proto['total_cycles'] = 0
    proto['current_cycles'] = 0
    return proto

def init():
    global state_cache
    if not os.path.isfile(STATE_FILE):
        with open(STATE_FILE, 'w') as f:
            state_cache = mkstate()
            json.dump(state_cache, f)
    else:
        with open(STATE_FILE, 'r') as f:
            state_cache = json.load(f)

def save():
    with open(STATE_FILE, 'w') as f:
        json.dump(state_cache, f)

def get():
    return state_cache

def incr(key):
    if key in state_cache.keys():
        if isinstance(state_cache[key], int):
            state_cache[key] += 1
            save()

if __name__ == '__main__':
    print(STATE_FILE)

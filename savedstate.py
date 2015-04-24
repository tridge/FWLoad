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
    try:
        f = open(STATE_FILE, 'r')
        state_cache = json.load(f)
    except Exception:
        f = open(STATE_FILE, 'w')
        state_cache = mkstate()
        json.dump(state_cache, f)
        pass

def save():
    try:
        f = open(STATE_FILE, 'w')
        json.dump(state_cache, f)
    except Exception:
        print("FAILED TO SAVE %s" % STATE_FILE)
        pass

def get():
    return state_cache

def incr(key):
    if key in state_cache.keys():
        if isinstance(state_cache[key], int):
            state_cache[key] += 1
            save()

def reset(key):
    if key in state_cache.keys():
        if isinstance(state_cache[key], int):
            state_cache[key] = 0
            save()

if __name__ == '__main__':
    print(STATE_FILE)

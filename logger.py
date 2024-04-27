#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr 20 16:14:31 2024

@author: mrityunjay
"""

import numpy as np
import logging

green = "\x1b[32m"
red = "\x1b[31;20m"
bold_red = "\x1b[31;1m"
reset = "\x1b[0m"

def initiate_log():
    level = logging.INFO
    format = '[%(levelname)s] - %(message)s'
    logging.basicConfig(level=level, format=format)
    
def debug(str):
    logging.debug(str)

def info(str):
    logging.info(green + str + reset)

def warn(str):
    logging.warning(red + str + reset)

def error(str):
    logging.error(bold_red + str + reset)
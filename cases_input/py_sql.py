#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jun 22 23:53:41 2024

@author: mrityunjay
"""

import mysql.connector

# Connect to MySQL server
mydb = mysql.connector.connect(
    host="localhost",
    user='root',
    password="Harsh@24",
    database='world' 
)
mycursor = mydb.cursor()

# Execute SQL queries
mycursor.execute("SELECT * FROM city")
result = mycursor.fetchall()

# Display the results
for row in result:
    print(row)

from pymongo import MongoClient
import pymongo
import re
import json

import requests

def fetch_username():
    try:
        response = requests.get('http://localhost:3000/api/user')  
        if response.status_code == 200:
            data = response.json()
            return data.get('username')
        else:
            print("Failed to fetch username")
            return None
    except requests.exceptions.RequestException as e:
        print(f"Error fetching username: {e}")
        return None


while True:
    username = fetch_username()
    if username:
        print(f"Username: {username}")
    else:
        print("No username found")
    username+="_waste_records"
    client = MongoClient("mongodb+srv://arjundevraj05:arjun123@cluster0.ev0ma.mongodb.net/?retryWrites=true&w=majority&appName=Cluster0")  
    db = client["safai"]
    collection = db[username]
    plastic = collection.count_documents({"Class": "PLASTIC"})
    paper = collection.count_documents({"Class": "PAPER"})
    metal = collection.count_documents({"Class": "METAL"})
    cardboard = collection.count_documents({"Class": "CARDBOARD"})
    glass = collection.count_documents({"Class": "GLASS"})
    biodegradable = collection.count_documents({"isBiodegradable": True})
    biodegradable_type = collection.count_documents({"isBiodegradable": False})
    nonbiodegradable_type = collection.count_documents({"isBiodegradable": False})
    count=collection.count_documents({})
    dict={"plastic":plastic,"paper":paper,"metal":metal,"cardboard":cardboard,"glass":glass,"biodegradable":biodegradable,"biodegradable_type":biodegradable_type,"nonbiodegradable_type":nonbiodegradable_type,"count":count}
    with open("stats.json",'w') as file:
        json.dump(dict,file)



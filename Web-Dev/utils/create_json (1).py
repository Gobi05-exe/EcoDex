from pymongo import MongoClient
import pymongo
import re
import json

while True:
    with open('WebDev\\utils\\username.ts', 'r') as file:
        content = file.read()
    match = re.search(r'\"(.*?)\"', content)
    if match:
        extracted_string = match.group(1)
        extracted_string+="_waste_records"
        print(extracted_string)
    else:
        print("No string found")
    client = MongoClient("mongodb+srv://arjundevraj05:arjun123@cluster0.ev0ma.mongodb.net/?retryWrites=true&w=majority&appName=Cluster0")  # Replace with your MongoDB URI
    db = client["safai"]
    collection = db[extracted_string]
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



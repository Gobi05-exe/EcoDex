// /app/api/data/count/route.ts
import { NextResponse } from 'next/server';
import { MongoClient } from 'mongodb';

const uri = process.env.MONGODB_URI; // Store MongoDB URI in environment variables
const client = new MongoClient(uri);

export async function GET() {
  try {
    await client.connect();
    const database = client.db('your_database_name'); // Replace with your DB name
    const collection = database.collection('your_collection_name'); // Replace with your collection name

    const plastic = await collection.countDocuments({ "Class": "PLASTIC" });
    const paper = await collection.countDocuments({ "Class": "PAPER" });
    const metal = await collection.countDocuments({ "Class": "METAL" });
    const cardboard = await collection.countDocuments({ "Class": "CARDBOARD" });
    const glass = await collection.countDocuments({ "Class": "GLASS" });
    const biodegradable = await collection.countDocuments({ "isBiodegradable": true });
    const nonbiodegradable = await collection.countDocuments({ "isBiodegradable": false });
    const count = await collection.countDocuments({});

    return NextResponse.json({
      plastic,
      paper,
      metal,
      cardboard,
      glass,
      biodegradable,
      nonbiodegradable,
      totalCount: count,
    });
  } catch (error) {
    console.error(error);
    return NextResponse.json({ error: 'Failed to fetch data' }, { status: 500 });
  } finally {
    await client.close();
  }
}

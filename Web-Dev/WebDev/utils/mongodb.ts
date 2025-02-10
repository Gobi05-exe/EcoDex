import { MongoClient, Db } from 'mongodb';

let client: MongoClient | null = null;
let db: Db | null = null;
const uri = process.env.MONGODB_URI as string; // MongoDB connection string from environment variables
const dbName = process.env.MONGODB_DB as string; // Database name from environment variables

if (!uri || !dbName) {
  throw new Error('Please define the MONGODB_URI and MONGODB_DB environment variables');
}

export async function connectToDatabase() {
  if (client && db) {
    // Return existing connection if available
    return { client, db };
  }

  // Otherwise, create a new client instance and connect
  client = new MongoClient(uri);
  await client.connect();
  db = client.db(dbName);

  console.log('Connected to MongoDB');

  return { client, db };
}

// /app/api/user/current/route.ts
import { NextRequest, NextResponse } from 'next/server';
import { connectToDatabase } from '@/utils/mongodb';
// Importing cookie/session library (for example, if you're using next-auth or a similar library)
// You can use Redis if you set up session storage with Redis

export async function GET(request: NextRequest) {
    try {
        // Assuming you have stored the username in a cookie or session
        const cookie = request.cookies.get('username'); // Fetch username from cookie

        if (!cookie) {
            return NextResponse.json({ error: 'Username not found' }, { status: 401 });
        }

        const username = cookie.value; // Extract username value from the cookie

        // Connect to MongoDB and retrieve documents
        const { db } = await connectToDatabase();
        const collectionName = `${username}_waste_records`;
        console.log("Collection name:", collectionName); // Debugging line

        const collection = db.collection(collectionName);
        const documents = await collection.find({}).toArray();
        
        return NextResponse.json(documents);
    } catch (error) {
        console.error('Error fetching records:', error);
        return NextResponse.json({ error: 'Failed to fetch records' }, { status: 500 });
    }
}

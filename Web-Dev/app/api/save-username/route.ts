// /app/api/save-username/route.ts
import { NextRequest, NextResponse } from 'next/server';

export async function POST(request: NextRequest) {
    try {
        const { username } = await request.json();
        
        if (!username) {
            return NextResponse.json({ error: 'Username is required' }, { status: 400 });
        }

        // Set the username in a cookie that lasts, for example, 1 day
        const response = NextResponse.json({ success: true });
        response.cookies.set('username', username, { httpOnly: true, maxAge: 24 * 60 * 60 }); // 1 day

        return response;
    } catch (error) {
        console.error('Error saving username:', error);
        return NextResponse.json({ error: 'Failed to save username' }, { status: 500 });
    }
}

'use client';
import { useState, useEffect } from "react";
import io from "socket.io-client";

// Replace with your Flask Ngrok URL
const socket = io("https://your-ngrok-url.ngrok-free.app");

export default function OverrideControl() {
    const [status, setStatus] = useState("IDLE");

    useEffect(() => {
        // Listen for security updates from Flask server
        socket.on("security-update", (data) => {
            setStatus(data.status);
        });

        return () => {
            socket.off("security-update");
        };
    }, []);

    const sendCommand = async (command: string) => {
        const res = await fetch("/api/override", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ command }),
        });

        const data = await res.json();
        if (data.status) setStatus(data.status);
    };

    return (
        <div className="p-5 bg-white shadow-lg rounded-lg text-center">
            <h2 className="text-2xl font-bold mb-4">Security Override</h2>
            <p className="text-lg font-semibold">Status: <span className="text-green-600">{status}</span></p>

            <div className="mt-4 flex justify-center gap-4">
                <button onClick={() => sendCommand("LOCK")} className="px-4 py-2 bg-red-500 text-white rounded-lg">Lock</button>
                <button onClick={() => sendCommand("UNLOCK")} className="px-4 py-2 bg-green-500 text-white rounded-lg">Unlock</button>
                <button onClick={() => sendCommand("ALERT")} className="px-4 py-2 bg-yellow-500 text-white rounded-lg">Alert</button>
            </div>
        </div>
    );
}

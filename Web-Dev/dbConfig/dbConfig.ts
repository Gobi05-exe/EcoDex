import mongoose from 'mongoose';

const MONGODB_URI = process.env.MONGODB_URI!;

if (!MONGODB_URI) {
  throw new Error("MONGODB_URI is not defined in environment variables");
}

export async function connect() {
  try {
    if (mongoose.connection.readyState >= 1) {
      console.log("Already connected to MongoDB");
      return;
    }

    await mongoose.connect(MONGODB_URI, {
      serverSelectionTimeoutMS: 10000, // 10 seconds timeout
    });

    mongoose.connection.on('connected', () => {
      console.log("MongoDB Connected");
    });

    mongoose.connection.on('error', (err) => {
      console.error("MongoDB Connection Error:", err);
      process.exit(1);
    });

  } catch (error) {
    console.error("Error in connecting to MongoDB:", error);
    process.exit(1);
  }
}

// utils/redis.ts
import Redis from 'ioredis';

const redis = new Redis({
  host: process.env.REDIS_HOST, // Replace with your Redis host
  port: parseInt(process.env.REDIS_PORT || "6379"), // Replace with your Redis port
  password: process.env.REDIS_PASSWORD, // Use your Redis password if applicable
});

export default redis;

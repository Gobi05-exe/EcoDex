# RAG-ED: IoT-driven Waste Management with RagBot

RAG-ED is an innovative platform designed to integrate with **RagBot**, an autonomous IoT robot capable of detecting, collecting, and segregating waste based on its type. This platform enhances waste management efficiency using advanced AI and IoT technologies.

## Features

- **RagBot Integration**: Connects to **RagBot**, an IoT-based waste-collecting robot that uses the **YOLO v8 object detection model** to autonomously sort waste into categories like:
  - Biodegradable
  - Cardboard
  - Cloth
  - Metal
  - Plastic
  - Glass
  - Paper
- **Custom AI Chatbot**: A smart, AI-powered chatbot built using **Chatbase** to assist users in interacting with the platform.
- **Waste Management Statistics**: Visualizes waste collection data using **Chart.js**, displaying interactive pie charts and line graphs.
- **Location Tracking with Leaflet Maps**: The website features a live map using the **Leaflet Maps API** to track RagBot's location and activity.
- **User-Friendly Interface**: Built using **Next.js** and **TailwindCSS** for a responsive, intuitive user interface.
- **Secure User Data Management**: All user data, along with records of waste collection and segregation, are securely stored in a **MongoDB** database.

## Tech Stack

- **Frontend**: 
  - Next.js
  - TailwindCSS
  - Leaflet Maps API for interactive maps
  - Chart.js for data visualization (pie charts and line graphs)
  
- **Backend**: 
  - API routes powered by Next.js for user authentication and data management
  - MongoDB for storing user data and waste collection records

- **AI & IoT**:
  - YOLO v8 object detection model integrated with RagBot for waste sorting
  - Custom AI chatbot built using Chatbase

## Setup Instructions

1. Clone the repository:
   ```bash
   git clone https://github.com/Arjundevraj05/EcoDex.git
   cd EcoDex

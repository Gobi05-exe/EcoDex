'use client';
import { useEffect, useRef } from 'react';
import L, { LatLngTuple } from 'leaflet';
import 'leaflet/dist/leaflet.css';

// Custom marker icon using a CDN
const customIcon = new L.Icon({
  iconUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-icon.png',
  shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-shadow.png',
  iconSize: [25, 41],
  iconAnchor: [12, 41],
  popupAnchor: [1, -34],
  shadowSize: [41, 41],
});

interface MapComponentProps {
  locations: { lat: number; lng: number }[];
}

const MapComponent: React.FC<MapComponentProps> = ({ locations }) => {
  const mapRef = useRef<L.Map | null>(null);

  useEffect(() => {
    if (mapRef.current) return; // Prevent reinitialization

    const map = L.map('map').setView([0, 0], 2);
    mapRef.current = map;

    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      attribution: '© OpenStreetMap contributors',
    }).addTo(map);

    const markers = L.featureGroup();

    locations.forEach(({ lat, lng }) => {
      const coordinates: LatLngTuple = [lat, lng];
      L.marker(coordinates, { icon: customIcon })
        .addTo(markers)
        .bindPopup(`Waste Location`);
    });

    markers.addTo(map);

    if (locations.length > 1) {
      map.fitBounds(markers.getBounds(), { padding: [50, 50] });
    } else if (locations.length === 1) {
      map.setView([locations[0].lat, locations[0].lng], 13);
    }

    return () => {
      map.remove();
      mapRef.current = null; // Cleanup when unmounting
    };
  }, [locations]);

  return <div id="map" className="h-full w-full" />;
};

export default MapComponent;

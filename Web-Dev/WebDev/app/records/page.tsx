'use client';

import { useEffect, useState } from 'react';

const RecordsPage = () => {
  const [records, setRecords] = useState<any[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [biodegradableCount, setBiodegradableCount] = useState(0);
  const [totalWasteCount, setTotalWasteCount] = useState(0);
  const [plasticCount, setPlasticCount] = useState(0);
  const [metalCount, setMetalCount] = useState(0);
  const [paperCount, setPaperCount] = useState(0);
  const [cardboardCount, setCardboardCount] = useState(0);
  const [glassCount, setGlassCount] = useState(0);

  useEffect(() => {
    const fetchRecords = async () => {
      try {
        const response = await fetch('/api/users/current'); // Adjust this path if necessary
        if (!response.ok) {
          throw new Error('Failed to fetch records');
        }
        const data = await response.json();

        // Calculate total waste count
        setTotalWasteCount(data.length);

        // Filter the records for biodegradable waste and calculate the count
        const biodegradableItems = data.filter((item: any) => item.isBiodegradable === true);
        setBiodegradableCount(biodegradableItems.length);

        // Calculate counts for specific waste types
        const plasticItems = data.filter((item: any) => item.Class === 'PLASTIC');
        const metalItems = data.filter((item: any) => item.Class === 'METAL');
        const paperItems = data.filter((item: any) => item.Class === 'PAPER');
        const cardboardItems = data.filter((item: any) => item.Class === 'CARDBOARD');
        const glassItems = data.filter((item: any) => item.Class === 'GLASS');

        setPlasticCount(plasticItems.length);
        setMetalCount(metalItems.length);
        setPaperCount(paperItems.length);
        setCardboardCount(cardboardItems.length);
        setGlassCount(glassItems.length);

        setRecords(data);
      } catch (err: any) {
        setError(err.message);
      } finally {
        setLoading(false);
      }
    };

    fetchRecords();
  }, []);

  if (loading) return <div>Loading...</div>;
  if (error) return <div>Error: {error}</div>;

  return (
    <div>
      <h1>Records Summary</h1>
      <p>Total Waste: {totalWasteCount}</p>
      <p>Total Biodegradable Waste: {biodegradableCount}</p>
      <p>Total Plastic Waste: {plasticCount}</p>
      <p>Total Metal Waste: {metalCount}</p>
      <p>Total Paper Waste: {paperCount}</p>
      <p>Total Cardboard Waste: {cardboardCount}</p>
      <p>Total Glass Waste: {glassCount}</p>
    </div>
  );
};

export default RecordsPage;

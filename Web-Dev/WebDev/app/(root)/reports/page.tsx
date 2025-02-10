'use client';
import React, { useEffect, useState } from 'react';
import { Doughnut, Line } from 'react-chartjs-2';
import { Chart as ChartJS, ArcElement, Tooltip, Legend, CategoryScale, LinearScale, PointElement, LineElement } from 'chart.js';
import CountUp from 'react-countup';

// Register chart components
ChartJS.register(ArcElement, Tooltip, Legend, CategoryScale, LinearScale, PointElement, LineElement);

const ReportsPage = () => {
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
    const fetchReports = async () => {
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

    fetchReports();
  }, []);

  if (loading) return <div>Loading...</div>;
  if (error) return <div>Error: {error}</div>;

  const doughnutData = {
    labels: ['Plastic', 'Metal', 'Paper', 'Cardboard', 'Glass', 'Biodegradable'],
    datasets: [
      {
        data: [plasticCount, metalCount, paperCount, cardboardCount, glassCount, biodegradableCount],
        backgroundColor: ['#4caf50', '#ff9800', '#03a9f4', '#8bc34a', '#f44336', '#00bcd4'],
        hoverBackgroundColor: ['#66bb6a', '#ffb74d', '#29b6f6', '#aed581', '#e57373', '#26c6da'],
      },
    ],
  };

  const lineData = {
    labels: ['Monday', 'Tuesday', 'Wednesday', 'Thursday', 'Friday', 'Saturday', 'Sunday'],
    datasets: [
      {
        label: 'Total Waste Collected',
        data: [8, 10, 4, 13, 12, 10, 8],
        fill: false,
        backgroundColor: '#4caf50',
        borderColor: '#4caf50',
      },
    ],
  };

  return (
    <div className="flex justify-center items-center min-h-screen bg-green-50">
      <div className="ml-40 mb-5 mt-5 w-full max-w-4xl bg-white p-8 shadow-lg rounded-2xl">
        <div>
          <h1 className="text-3xl font-bold font-stacker text-center border-b-1">RAG-ED</h1>
          <p className="text-4xl font-bold font-libre mb-8 text-center">Performance Report</p>
        </div>

        {/* Stats Cards */}
        <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 gap-6 mb-8">
          <StatsCard title="Waste Items Collected" value={totalWasteCount} />
          <StatsCard title="Biodegradable Waste Collected" value={biodegradableCount} />
          <StatsCard title="Non-Biodegradable Waste Collected" value={totalWasteCount - biodegradableCount} />
        </div>

        {/* Charts */}
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-8">
          {/* Doughnut Chart for waste breakdown */}
          <div className="p-6 bg-gray-50 shadow-md rounded-lg">
            <h2 className="text-lg font-semibold mb-4 text-center">Waste Breakdown</h2>
            <Doughnut data={doughnutData} />
          </div>

          {/* Line Graph for time-based trends */}
          <div className="p-6 bg-gray-50 shadow-md rounded-lg">
            <h2 className="text-lg font-semibold mb-4 text-center">Collection Trends</h2>
            <div className="h-72"> {/* Adjust height to reduce whitespace */}
              <Line data={lineData} options={{ maintainAspectRatio: false }} />
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

// StatsCard Component for displaying stats
const StatsCard = ({ title, value }: { title: string; value: number }) => (
  <div className="cursor-pointer flex flex-row items-center bg-gradient-to-b from-green-300 to-green-500 rounded-lg p-1 hover:scale-105 transition-transform duration-200 mt-3 h-40">
    <div className="flex items-center bg-gray-50 rounded-lg p-5 w-full h-full">
      <div className="ml-3 flex-grow">
        <p className="pr-4 min-h-20 text-center font-poppins font-semibold text-xl bg-gradient-to-r from-green-600 to-green-800 text-transparent bg-clip-text">
          {title}
        </p>
        <p className="mr-5 text-center font-poppins font-bold text-3xl bg-gradient-to-r from-green-600 to-green-800 text-transparent bg-clip-text">
          <CountUp end={value} duration={2.5} />
        </p>
      </div>
    </div>
  </div>
);

export default ReportsPage;

"use client"
import React, { useEffect, useState } from 'react';
import Image from 'next/image';
import CountUp from 'react-countup';
import { Coins } from 'lucide-react';

const Status1 = () => {
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
  const [tokensEarned, setTokensEarned] = useState(0);
  const [carbonReduced, setCarbonReduced] = useState(0);

  useEffect(() => {
    const fetchReports = async () => {
      try {
        const response = await fetch('/api/users/current'); // Adjust this path if necessary
        if (!response.ok) {
          throw new Error('Failed to fetch records');
        }
        const data = await response.json();

        setTotalWasteCount(data.length);

        const biodegradableItems = data.filter((item: any) => item.isBiodegradable === true);
        setBiodegradableCount(biodegradableItems.length);

        const plasticItems = data.filter((item: any) => item.Class === 'PLASTIC');
        const metalItems = data.filter((item: any) => item.Class === 'METAL');
        const paperItems = data.filter((item: any) => item.Class === 'PAPER');
        const cardboardItems = data.filter((item: any) => item.Class === 'CARDBOARD');
        const glassItems = data.filter((item: any) => item.Class === 'GLASS');

        const plasticCount = plasticItems.length;
        const metalCount = metalItems.length;
        const paperCount = paperItems.length;
        const cardboardCount = cardboardItems.length;
        const glassCount = glassItems.length;

        setPlasticCount(plasticCount);
        setMetalCount(metalCount);
        setPaperCount(paperCount);
        setCardboardCount(cardboardCount);
        setGlassCount(glassCount);

        // Calculate Tokens Earned
        const tokens =
          plasticCount * 2 +
          metalCount * 3 +
          paperCount * 1 +
          cardboardCount * 2 +
          glassCount * 2;
        setTokensEarned(tokens);

        // Calculate Carbon Emissions Reduced
        const emissions =
          plasticCount * 6 +
          metalCount * 10 +
          paperCount * 4 +
          cardboardCount * 5 +
          glassCount * 3;
        setCarbonReduced(emissions);

        setRecords(data);
      } catch (err: any) {
        setError(err.message);
      } finally {
        setLoading(false);
      }
    };

    fetchReports();
  }, []);

  return (
    <div className='mb-6 mt-10 flex justify-center flex-row gap-3 flex-wrap'>

      {/* Biodegradable Section */}
      <div className='cursor-pointer flex flex-row items-center bg-gradient-to-b from-green-300 to-green-500 rounded-lg p-[2px] hover:scale-105 transition-transform duration-200 w-72 h-36'>
        <div className='flex items-center bg-gray-50 rounded-lg p-5 w-full h-full'>
          <div className='rounded-full p-3 bg-gradient-to-b from-green-100 to-green-200 flex-shrink-0'>
            <Image src='/icons/biodegradable.svg' width={50} height={50} alt='Biodegradable Icon' />
          </div>
          <div className='ml-4 flex-grow'>
            <p className='font-poppins font-semibold text-xl bg-gradient-to-r from-green-600 to-green-800 text-transparent bg-clip-text'>
              Biodegradable
            </p>
            <p className='font-poppins font-bold text-3xl mt-2 bg-gradient-to-r from-green-600 to-green-800 text-transparent bg-clip-text'>
              <CountUp end={biodegradableCount} duration={2.5} />
            </p>
          </div>
        </div>
      </div>

      {/* Non-Biodegradable Section */}
      <div className='cursor-pointer flex flex-row items-center bg-gradient-to-b from-green-300 to-green-500 rounded-lg p-[2px] hover:scale-105 transition-transform duration-200 w-72 h-36'>
        <div className='flex items-center bg-gray-50 rounded-lg p-5 w-full h-full'>
          <div className='rounded-full p-3 bg-gradient-to-b from-green-100 to-green-200 flex-shrink-0'>
            <Image src='/icons/nonbiodegradable.svg' width={50} height={50} alt='Non-Biodegradable Icon' />
          </div>
          <div className='ml-4 flex-grow'>
            <p className='font-poppins font-semibold text-xl bg-gradient-to-r from-green-600 to-green-800 text-transparent bg-clip-text'>
              Non-Biodegradable
            </p>
            <p className='font-poppins font-bold text-3xl mt-2 bg-gradient-to-r from-green-600 to-green-800 text-transparent bg-clip-text'>
              <CountUp end={totalWasteCount - biodegradableCount} duration={2.5} />
            </p>
          </div>
        </div>
      </div>

      {/* Carbon Emissions Reduced Section */}
      <div className='cursor-pointer flex flex-row items-center bg-gradient-to-b from-green-300 to-green-500 rounded-lg p-[2px] hover:scale-105 transition-transform duration-200 w-72 h-36'>
        <div className='flex items-center bg-gray-50 rounded-lg p-5 w-full h-full'>
          <div className='rounded-full p-3 bg-gradient-to-b from-green-100 to-green-200 flex-shrink-0'>
            <Image src='/icons/carbon1.svg' width={50} height={50} alt='Carbon Emissions Icon' />
          </div>
          <div className='ml-4 flex-grow'>
            <p className='font-poppins font-semibold text-xl bg-gradient-to-r from-green-600 to-green-800 text-transparent bg-clip-text'>
              Carbon Emissions Reduced
            </p>
            <p className='font-poppins font-bold text-3xl bg-gradient-to-r from-green-600 to-green-800 text-transparent bg-clip-text'>
              <CountUp end={carbonReduced} duration={2.5} />
            </p>
          </div>
        </div>
      </div>

      {/* Tokens Earned Section */}
      <div className='cursor-pointer flex flex-row items-center bg-gradient-to-b from-green-300 to-green-500 rounded-lg p-[2px] hover:scale-105 transition-transform duration-200 w-72 h-36'>
        <div className='flex items-center bg-gray-50 rounded-lg p-5 w-full h-full'>
          <div className='rounded-full p-3 bg-gradient-to-b from-green-100 to-green-200 flex-shrink-0'>
            <Coins className="h-12 w-12 text-green-600" />
          </div>
          <div className='ml-4 flex-grow'>
            <p className='font-poppins font-semibold text-xl bg-gradient-to-r from-green-600 to-green-800 text-transparent bg-clip-text'>
              Tokens Earned
            </p>
            <p className='font-poppins font-bold text-3xl bg-gradient-to-r from-green-600 to-green-800 text-transparent bg-clip-text'>
              <CountUp end={tokensEarned} duration={2.5} />
            </p>
          </div>
        </div>
      </div>
    </div>
  );
};

export default Status1;

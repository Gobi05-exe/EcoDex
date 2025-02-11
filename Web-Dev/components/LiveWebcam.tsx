'use client';

import React, { useEffect, useRef, useState } from 'react';
import { Camera, ArrowUp, ArrowDown, ArrowLeft, ArrowRight } from 'lucide-react';

const LiveWebcam: React.FC = () => {
  const videoRef = useRef<HTMLVideoElement>(null);
  const [activeKey, setActiveKey] = useState<string | null>(null);

  useEffect(() => {
    const getWebcamFeed = async () => {
      try {
        const stream = await navigator.mediaDevices.getUserMedia({ video: true });
        if (videoRef.current) {
          videoRef.current.srcObject = stream;
        }
      } catch (err) {
        console.error('Error accessing webcam:', err);
      }
    };

    getWebcamFeed();
  }, []);

  const handleKeyPress = (e: KeyboardEvent) => {
    if (['w', 'a', 's', 'd'].includes(e.key)) {
      setActiveKey(e.key);
      console.log(`Key pressed: ${e.key.toUpperCase()}`);
      // Here you can add the function to control the robot
    }
  };

  const handleKeyRelease = () => {
    setActiveKey(null);
  };

  useEffect(() => {
    window.addEventListener('keydown', handleKeyPress);
    window.addEventListener('keyup', handleKeyRelease);
    return () => {
      window.removeEventListener('keydown', handleKeyPress);
      window.removeEventListener('keyup', handleKeyRelease);
    };
  }, []);

  const handleButtonClick = (key: string) => {
    setActiveKey(key);
    console.log(`Button clicked: ${key.toUpperCase()}`);
    // You can trigger manual control of the robot here as well
  };

  return (
    <div className='ml-40 flex flex-col items-center justify-center h-screen bg-gradient-to-r from-green-50 to-white py-4'> {/* Added padding for top and bottom */}
      {/* Card Container */}
      <div className='relative w-full max-w-2xl p-4 rounded-lg shadow-lg bg-white'> {/* Reduced padding */}
        {/* Webcam Feed */}
        <div className='relative w-full h-80 rounded-lg overflow-hidden mb-4 border border-gray-200'>
          <video ref={videoRef} autoPlay playsInline className="w-full h-full object-cover" />
        </div>

        {/* Live Webcam Feed Text */}
        <div className='flex flex-row items-center justify-center mb-4'>
          <Camera className="h-6 w-6 text-green-600 mr-2" />
          <p className='font-poppins font-semibold text-xl bg-gradient-to-r from-green-600 to-green-800 text-transparent bg-clip-text'>
            Live Webcam Feed
          </p>
        </div>

        {/* Control Buttons */}
        <div className='flex flex-col items-center'>
          {/* Up Arrow */}
          <button
            onMouseDown={() => handleButtonClick('w')}
            onMouseUp={handleKeyRelease}
            className={`p-2 mb-2 rounded-full bg-green-300 hover:bg-green-400 transition ${activeKey === 'w' ? 'scale-95' : ''}`} // Reduced padding
          >
            <ArrowUp className="h-8 w-8 text-white" />
          </button>
          <div className='flex mb-2'>
            {/* Left Arrow */}
            <button
              onMouseDown={() => handleButtonClick('a')}
              onMouseUp={handleKeyRelease}
              className={`p-2 m-2 rounded-full bg-green-300 hover:bg-green-400 transition ${activeKey === 'a' ? 'scale-95' : ''}`} // Reduced padding
            >
              <ArrowLeft className="h-8 w-8 text-white" />
            </button>
            {/* Down Arrow */}
            <button
              onMouseDown={() => handleButtonClick('s')}
              onMouseUp={handleKeyRelease}
              className={`p-2 m-2 rounded-full bg-green-300 hover:bg-green-400 transition ${activeKey === 's' ? 'scale-95' : ''}`} // Reduced padding
            >
              <ArrowDown className="h-8 w-8 text-white" />
            </button>
            {/* Right Arrow */}
            <button
              onMouseDown={() => handleButtonClick('d')}
              onMouseUp={handleKeyRelease}
              className={`p-2 m-2 rounded-full bg-green-300 hover:bg-green-400 transition ${activeKey === 'd' ? 'scale-95' : ''}`} // Reduced padding
            >
              <ArrowRight className="h-8 w-8 text-white" />
            </button>
          </div>
        </div>
      </div>
    </div>
  );
};

export default LiveWebcam;

// components/HeaderBox.tsx
'use client';

import React from 'react';
import { Montserrat, Lato } from 'next/font/google';
import { useEffect, useState } from 'react';

const montserrat = Montserrat({
  subsets: ['latin'],
  weight: ['400', '600', '700'],
});

const lato = Lato({
  subsets: ['latin'],
  weight: ['400', '700'],
});



const HeaderBox: React.FC  = () => {
    const [username, setUsername] = useState<string | null>(null);
    useEffect(() => {
        const fetchUsername = async () => {
            try {
                const response = await fetch('/api/user');
                if (response.ok) {
                    const data = await response.json();
                    setUsername(data.username);
                } else {
                    console.error('Failed to fetch username');
                }
            } catch (error) {
                console.error('Error fetching username:', error);
            }
        };

        fetchUsername();
    }, []);
  return (
    <div className=" py-6 ml-56 flex flex-col w-4/5 justify-start gap-1 md:flex-row items-center border-b pb-5">
      <div>
        <h1 className='text-24 lg:text-30 font-poppins font-semibold text-gray-800'>
          Welcome Back,  
          <span className='bg-gradient-to-r from-green-400 to-green-600 text-transparent bg-clip-text font-poppins'>&nbsp;{username ? `Welcome, ${username}!` : 'Welcome, Guest!'}</span>!
        </h1>
        <p className="text-14 lg:text-16 font-normal text-gray-600">
          Ready to make a difference? Let’s clean up the world together.
        </p>
      </div>
    </div>
  );
};

export default HeaderBox;

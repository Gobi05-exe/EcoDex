'use client';

import React, { useEffect, useState } from 'react';
import { Montserrat, Lato } from 'next/font/google';
import { fetchUsername } from '../utils/fetchUsername';
import { motion } from 'framer-motion';

// Font imports
const montserrat = Montserrat({
  subsets: ['latin'],
  weight: ['400', '600', '700'],
});

const lato = Lato({
  subsets: ['latin'],
  weight: ['400', '700'],
});

const HeaderBox: React.FC = () => {
  const [username, setUsername] = useState<string | null>(null);

  // Fetch username on mount
  useEffect(() => {
    const getUsername = async () => {
      const fetchedUsername = await fetchUsername();
      setUsername(fetchedUsername);
    };

    getUsername();
  }, []);

  // Framer Motion animation variants for the text
  const textVariants = {
    hidden: { opacity: 0, y: -20 },
    visible: {
      opacity: 1,
      y: 0,
      transition: { duration: 0.6, ease: 'easeOut' },
    },
  };

  // Subtext animation
  const subTextVariants = {
    hidden: { opacity: 0, y: 10 },
    visible: {
      opacity: 1,
      y: 0,
      transition: { duration: 0.6, ease: 'easeOut', delay: 0.3 },
    },
  };

  return (
    <div className="py-6 ml-56 flex flex-col w-4/5 justify-start gap-1 md:flex-row items-center border-b pb-5">
      <motion.div
        initial="hidden"
        animate="visible"
        variants={textVariants}
        className="header-box"
      >
        <h1 className="text-24 lg:text-30 font-poppins font-semibold text-gray-800">
          Welcome Back,{' '}
          <span className="bg-gradient-to-r from-green-400 to-green-600 text-transparent bg-clip-text font-poppins">
            &nbsp;{username ? `${username}` : 'Guest'}
          </span>
          !
        </h1>
        <motion.p
          initial="hidden"
          animate="visible"
          variants={subTextVariants}
          className="text-14 lg:text-16 font-normal text-gray-600 mt-2"
        >
          Ready to make a difference? Let’s clean up the world together.
        </motion.p>
      </motion.div>
    </div>
  );
};

export default HeaderBox;

'use client';
import React, { useState, useEffect } from 'react';
import Image from 'next/image';
import {
  HomeIcon,
  DocumentReportIcon,
  MapIcon,
  CameraIcon,
  MenuAlt2Icon,
  XIcon,
  UserCircleIcon, // Imported user icon for profile picture
} from '@heroicons/react/outline';
import { useRouter } from 'next/navigation';
import LogoutButton from './LogoutButton'; 
import { fetchUsername } from '../utils/fetchUsername'; 

const Sidebar: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [loaded, setLoaded] = useState(false);
  const [activeItem, setActiveItem] = useState('Home');
  const [username, setUsername] = useState<string | null>(null);
  const router = useRouter();

  useEffect(() => {
    // Set loaded to true after component mounts
    setLoaded(true);

    // Fetch username on component mount
    const getUsername = async () => {
      const fetchedUsername = await fetchUsername();
      setUsername(fetchedUsername);
    };
    getUsername();
  }, []);

  // Navigation links
  const navigation = [
    { name: 'Home', icon: HomeIcon, href: '/' },
    { name: 'Reports', icon: DocumentReportIcon, href: '/reports' },
    { name: 'Map', icon: MapIcon, href: '/map' },
    { name: 'Live Camera', icon: CameraIcon, href: '/livecam' },
  ];

  // Handle navigation
  const handleNavigation = (item: string, href: string) => {
    setActiveItem(item);
    router.push(href);
  };

  return (
    <>
      {/* Mobile Header */}
      <div className="md:hidden flex items-center justify-between w-full bg-gray-50 text-black px-4 py-3 shadow-md">
        <button
          onClick={() => setIsOpen(!isOpen)}
          aria-label="Toggle Sidebar"
          className="focus:outline-none"
        >
          {isOpen ? <XIcon className="h-6 w-6" /> : <MenuAlt2Icon className="h-6 w-6" />}
        </button>
        <div className="flex items-center">
          <Image src="/icons/logo_main.svg" width={34} height={34} alt="logo" />
          <span className="ml-2 font-bold font-stacker text-xl">EcoDexHub</span>
        </div>
      </div>

      {/* Sidebar */}
      <aside
        className={`fixed inset-y-0 left-0 transform transition-transform duration-500 ease-in-out ${
          loaded ? 'translate-x-0' : '-translate-x-full'
        } ${isOpen ? 'md:translate-x-0' : '-translate-x-full'} md:translate-x-0 bg-gray-50 text-black w-52 z-50 border-r-2 shadow-lg`}
      >
        <div className="flex flex-col h-full p-6 border-b-1">
          {/* Logo Section */}
          <div className="flex items-center mb-8 border-b border-gray-300 pb-4">
            <Image src="/icons/logo_main.svg" width={34} height={34} alt="logo" />
            <span className="ml-2 text-lg font-stacker font-bold">Eco-Dex</span>
          </div>

          {/* Navigation */}
          <nav className="flex-1">
            <ul className="space-y-1">
              {navigation.map((item) => (
                <li key={item.name}>
                  <button
                    onClick={() => handleNavigation(item.name, item.href)}
                    className={`flex items-center w-full px-4 py-3 text-medium font-medium transition-all duration-200 rounded-md ${
                      activeItem === item.name
                        ? 'bg-green-100 text-green-600'
                        : 'text-gray-700 hover:bg-gray-100 hover:text-green-600'
                    }`}
                  >
                    <item.icon
                      className={`h-6 w-6 ${
                        activeItem === item.name ? 'text-green-600' : 'text-gray-500'
                      }`}
                    />
                    <span className="ml-2">{item.name}</span>
                  </button>
                </li>
              ))}
            </ul>
          </nav>

          {/* Logout Button */}
          <LogoutButton />

          {/* User Profile */}
          <div className="mt-8 flex items-center border-t border-grey-300 pt-4 space-x-4">
            {/* User Icon */}
            <UserCircleIcon className="h-10 w-10 text-gray-500" />
            <div>
              <p className="text-sm font-medium">{username ? username : 'Guest'}</p>
              <a href="#" className="text-xs text-gray-500 hover:underline">
                View Profile
              </a>
            </div>
          </div>
        </div>
      </aside>

      {/* Overlay for Mobile */}
      {isOpen && (
        <div
          className="fixed inset-0 bg-black opacity-50 z-40 md:hidden"
          onClick={() => setIsOpen(false)}
          aria-hidden="true"
        ></div>
      )}
    </>
  );
};

export default Sidebar;

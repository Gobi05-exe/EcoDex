'use client';
import React, { useState } from 'react';
import { FaRobot, FaRegChartBar, FaBell, FaArrowLeft, FaArrowRight, FaRecycle, FaTrash } from 'react-icons/fa';
import Chatbot from './Chatbot'; // Import the Chatbot component

const RightSidebar = () => {
    const [isExpanded, setIsExpanded] = useState(false); // Controls sidebar expansion

    return (
        <div
            className={`fixed inset-y-0 right-0 transform transition-transform duration-300 ease-in-out bg-white shadow-lg z-100 ${
                isExpanded ? 'translate-x-0 w-48' : 'translate-x-[5%] w-14'
            } border-l`}
            style={{ zIndex: 1000 }} // Ensure the sidebar stays visible
        >
            {/* Toggle Button */}
            <div
                className="absolute top-1/2 transform -translate-y-1/2 left-[-16px] bg-green-500 p-2 rounded-full cursor-pointer shadow-lg hover:bg-green-600 transition-colors"
                onClick={() => setIsExpanded(!isExpanded)}
            >
                {isExpanded ? <FaArrowRight className="text-white" /> : <FaArrowLeft className="text-white" />}
            </div>

            {/* Sidebar Content when expanded */}
            <div className={`p-4 ${isExpanded ? 'block' : 'hidden'} transition-opacity duration-300`}>
                {/* Bot Status Section */}
                <div className="mb-6 border-b pb-4">
                    <h2 className="font-semibold text-base text-green-700 flex items-center mb-3 font-poppins">
                        <FaRobot className="mr-2 text-green-700" /> Bot Status
                    </h2>
                    <p className="text-sm font-light font-poppins text-gray-700">
                        Status: <span className="font-medium text-green-700">Active</span>
                    </p>
                </div>

                {/* Waste Collection Section */}
                <div className="mb-6 border-b pb-4">
                    <h2 className="font-semibold text-base text-green-700 flex items-center mb-3 font-poppins">
                        <FaTrash className="mr-2 text-green-700" /> Waste Collection
                    </h2>
                    <p className="text-sm font-light font-poppins text-gray-700">
                        Plastic: <span className="font-medium text-green-700">20 kg</span>
                    </p>
                    <p className="text-sm font-light font-poppins text-gray-700">
                        Metal: <span className="font-medium text-green-700">10 kg</span>
                    </p>
                </div>

                {/* Alerts Section */}
                <div className="mb-6 border-b pb-4">
                    <h2 className="font-semibold text-base text-green-700 flex items-center mb-3 font-poppins">
                        <FaBell className="mr-2 text-green-700" /> Alerts
                    </h2>
                    <p className="text-sm font-light font-poppins text-gray-700">No alerts at this time.</p>
                </div>

                {/* Upcoming Section */}
                <div className="mb-6 border-b pb-4">
                    <h2 className="font-semibold text-base text-green-700 flex items-center mb-3 font-poppins">
                        <FaRegChartBar className="mr-2 text-green-700" /> Upcoming
                    </h2>
                    <p className="text-sm font-light font-poppins text-gray-700">
                        Next mission: <span className="font-medium text-green-700">Sector A at 3 PM</span>
                    </p>
                </div>

                {/* Tips Section */}
                <div className="mb-6 border-b pb-4">
                    <h2 className="font-semibold text-base text-green-700 flex items-center mb-3 font-poppins">
                        <FaRecycle className="mr-2 text-green-700" /> Tips
                    </h2>
                    <p className="text-sm font-light font-poppins text-gray-700">Recycle plastics whenever possible!</p>
                </div>
            </div>

            {/* Sidebar Icons in Collapsed Mode */}
            <div className={`p-4 ${!isExpanded ? 'block' : 'hidden'} transition-opacity duration-300`}>
                <div className="mb-6 flex justify-center">
                    <FaRobot className="text-green-700 text-xl hover:scale-125 hover:text-green-600 transition-transform duration-200 ease-in-out" />
                </div>
                <div className="mb-6 flex justify-center">
                    <FaTrash className="text-green-700 text-xl hover:scale-125 hover:text-green-600 transition-transform duration-200 ease-in-out" />
                </div>
                <div className="mb-6 flex justify-center">
                    <FaBell className="text-green-700 text-xl hover:scale-125 hover:text-green-600 transition-transform duration-200 ease-in-out" />
                </div>
                <div className="mb-6 flex justify-center">
                    <FaRegChartBar className="text-green-700 text-xl hover:scale-125 hover:text-green-600 transition-transform duration-200 ease-in-out" />
                </div>
                <div className="mb-6 flex justify-center">
                    <FaRecycle className="text-green-700 text-xl hover:scale-125 hover:text-green-600 transition-transform duration-200 ease-in-out" />
                </div>
            </div>

            {/* Add the Chatbot Component at the bottom of the sidebar */}
            <div className={`p-4 ${isExpanded ? 'block' : 'hidden'} transition-opacity duration-300`}>
                <Chatbot />
            </div>
        </div>
    );
};

export default RightSidebar;

import OverrideControl from "@/components/OverrideControl";

export default function ReportsPage() {
    return (
        <div className="flex flex-col items-center justify-center min-h-screen bg-gray-100">
            <h1 className="text-3xl font-bold mb-4">Reports & Security</h1>
            
            {/* Security Override Panel */}
            <OverrideControl />
        </div>
    );
}

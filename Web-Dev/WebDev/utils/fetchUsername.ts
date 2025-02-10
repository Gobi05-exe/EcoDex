export const fetchUsername = async () => {
    // Fetch the username
    const response = await fetch('/api/user');
    const data = await response.json();
    const username = data.username;

    // Call the save-username API route to export the username
    await fetch('/api/save-username', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({ username }),
    });

    return username;
};

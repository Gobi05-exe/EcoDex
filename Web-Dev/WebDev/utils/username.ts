// utils/username.ts

import { fetchUsername } from './fetchUsername';

export const getUsername = async (): Promise<string | null> => {
    const username = await fetchUsername();
    return username;
};

export default getUsername;

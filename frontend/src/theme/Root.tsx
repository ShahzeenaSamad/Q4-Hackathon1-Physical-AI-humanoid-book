import React, { ReactNode } from 'react';
import ChatbotWidget from '@site/src/components/ChatbotWidget';
import { AuthProvider } from '@site/src/components/Auth/AuthContext';

interface RootProps {
  children: ReactNode;
}

const Root: React.FC<RootProps> = ({ children }) => {
  return (
    <AuthProvider>
      {children}
      <ChatbotWidget />
    </AuthProvider>
  );
};

export default Root;

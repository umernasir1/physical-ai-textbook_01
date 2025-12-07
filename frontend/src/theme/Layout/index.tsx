import React, {type ReactNode} from 'react';
import Layout from '@theme-original/Layout';
import type LayoutType from '@theme/Layout';
import type {WrapperProps} from '@docusaurus/types';
import { AuthProvider } from '../../components/AuthProvider'; // Import AuthProvider
import Chatbot from '../../components/Chatbot'; // Import Chatbot

type Props = WrapperProps<typeof LayoutType>;

export default function LayoutWrapper(props: Props): ReactNode {
  return (
    <AuthProvider> {/* Wrap with AuthProvider */}
      <Layout {...props} />
      <div style={{ position: 'fixed', bottom: '20px', right: '20px', zIndex: 1000 }}>
        <Chatbot />
      </div>
    </AuthProvider>
  );
}

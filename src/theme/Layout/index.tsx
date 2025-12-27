import React from 'react';
import type {WrapperProps} from '@docusaurus/types';
import Layout from '@theme-original/Layout';
import RagChatbotPanel from '@site/src/components/RagChatbotPanel';
import { AuthProvider } from '@site/src/contexts/AuthContext';

type Props = WrapperProps<typeof Layout>;

export default function LayoutWrapper(props: Props): JSX.Element {
  return (
    <AuthProvider>
      <Layout {...props}>
        {props.children}
        <RagChatbotPanel />
      </Layout>
    </AuthProvider>
  );
}
import React from 'react';
import type {WrapperProps} from '@docusaurus/types';
import Layout from '@theme-original/Layout';
import RagChatbotPanel from '@site/src/components/RagChatbotPanel';

type Props = WrapperProps<typeof Layout>;

export default function LayoutWrapper(props: Props): JSX.Element {
  return (
    <>
      <Layout {...props}>
        {props.children}
        <RagChatbotPanel />
      </Layout>
    </>
  );
}
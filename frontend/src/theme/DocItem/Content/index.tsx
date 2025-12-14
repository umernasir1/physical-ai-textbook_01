import React, {useRef, type ReactNode} from 'react';
import clsx from 'clsx';
import {ThemeClassNames} from '@docusaurus/theme-common';
import {useDoc} from '@docusaurus/plugin-content-docs/client';
import Heading from '@theme/Heading';
import MDXContent from '@theme/MDXContent';
import type {Props} from '@docusaurus/theme-DocItem/Content';
import { useAuth } from '../../../components/AuthProvider';
import { useTranslation } from '../../../components/TranslationProvider';
import InlineTranslator from '../../../components/InlineTranslator';
import styles from './TranslatedContent.module.css';

/**
 Title can be declared inside md content or declared through
 front matter and added manually. To make both cases consistent,
 the added title is added under the same div.markdown block
 See https://github.com/facebook/docusaurus/pull/4882#issuecomment-853021120

 We render a "synthetic title" if:
 - user doesn't ask to hide it with front matter
 - the markdown content does not already contain a top-level h1 heading
*/
function useSyntheticTitle(): string | null {
  const {metadata, frontMatter, contentTitle} = useDoc();
  const shouldRender =
    !frontMatter.hide_title && typeof contentTitle === 'undefined';
  if (!shouldRender) {
    return null;
  }
  return metadata.title;
}

export default function DocItemContent({children}: Props): ReactNode {
  const { isAuthenticated, user } = useAuth();
  const { isTranslated, translatedContent } = useTranslation();
  const syntheticTitle = useSyntheticTitle();
  const contentRef = useRef<HTMLDivElement>(null);

  return (
    <>
      <InlineTranslator contentRef={contentRef} />

      <div
        ref={contentRef}
        className={clsx(ThemeClassNames.docs.docMarkdown, 'markdown')}
      >
        {!isTranslated ? (
          <>
            {syntheticTitle && (
              <header>
                <Heading as="h1">{syntheticTitle}</Heading>
              </header>
            )}

            {isAuthenticated && user && user.profile_data && (
              <div style={{border: '1px solid blue', padding: '1rem', marginBottom: '1rem'}}>
                Welcome back, {user.email}! This content is personalized for you based on your profile.
              </div>
            )}

            <MDXContent>{children}</MDXContent>
          </>
        ) : (
          <div className={styles.translatedContent}>
            {translatedContent.split('\n\n').map((paragraph, index) => (
              <p key={index}>{paragraph}</p>
            ))}
          </div>
        )}
      </div>
    </>
  );
}

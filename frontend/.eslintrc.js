module.exports = {
  "env": {
    "browser": true,
    "es2021": true,
    "node": true // Add node environment for 'module', 'require', 'process'
  },
  "extends": [
    "eslint:recommended",
    "plugin:react/recommended",
  ],
  "parserOptions": {
    "ecmaFeatures": {
      "jsx": true
    },
    "ecmaVersion": 12,
    "sourceType": "module",
    "requireConfigFile": false // Allow using parser without a separate config file
  },
  "plugins": [
    "react",
  ],
  "rules": {
    "react/react-in-jsx-scope": "off",
    "react/prop-types": "off", // Disable prop-types as we're using TypeScript
    "no-undef": "off", // Disable no-undef as TypeScript handles this in TS files, and Node handles in JS config files.
    "react/no-unescaped-entities": "off" // To ignore the error about apostrophes
  },
  "settings": {
    "react": {
      "version": "detect" // Automatically detect the React version
    }
  },
  "overrides": [
    {
      "files": ["**/*.ts", "**/*.tsx"], // Apply these settings to TypeScript files
      "extends": [
        "plugin:@typescript-eslint/recommended",
        "plugin:@typescript-eslint/recommended-requiring-type-checking"
      ],
      "parser": "@typescript-eslint/parser",
      "parserOptions": {
        "project": ["./tsconfig.json"], // Specify your tsconfig.json for type-aware linting
        "tsconfigRootDir": __dirname // Important for monorepos or complex project structures
      },
      "rules": {
        "@typescript-eslint/explicit-module-boundary-types": "off", // Adjust as needed
        "@typescript-eslint/no-var-requires": "off", // Allow require for Docusaurus config
        "@typescript-eslint/no-unsafe-assignment": "off", // Disable for Docusaurus generated code
        "@typescript-eslint/no-unsafe-call": "off",      // Disable for Docusaurus generated code
        "@typescript-eslint/no-unsafe-member-access": "off", // Disable for Docusaurus generated code
        "@typescript-eslint/no-unsafe-return": "off",    // Disable for Docusaurus generated code
        "@typescript-eslint/no-unsafe-argument": "off"   // Disable for Docusaurus generated code
      }
    }
  ]
};
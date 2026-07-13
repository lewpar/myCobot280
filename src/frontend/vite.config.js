import { defineConfig, loadEnv } from 'vite'
import react from '@vitejs/plugin-react'

export default defineConfig(({ mode }) => {
  const env = loadEnv(mode, process.cwd(), '')
  const proxyTarget = env.VITE_API_URL || 'http://localhost:8000'
  // Only use proxy when VITE_API_URL is not an external URL
  const useProxy = !env.VITE_API_URL || env.VITE_API_URL.startsWith('/')

  return {
    plugins: [react()],
    server: useProxy ? {
      proxy: {
        '/api': proxyTarget,
      },
    } : {},
  }
})

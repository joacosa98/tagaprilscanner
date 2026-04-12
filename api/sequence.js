import { get, put } from '@vercel/blob';

const BLOB_PATH = 'last-sequence.json';

export async function GET() {
  try {
    const result = await get(BLOB_PATH, { access: 'private' });
    if (!result || result.statusCode !== 200 || !result.stream) {
      return new Response('Not found', { status: 404 });
    }
    const text = await new Response(result.stream).text();
    return new Response(text, {
      headers: { 'Content-Type': 'application/json' },
    });
  } catch (error) {
    return new Response('Error reading blob', { status: 500 });
  }
}

export async function POST(request) {
  let data;
  try {
    data = await request.json();
  } catch (error) {
    return new Response('Invalid JSON', { status: 400 });
  }

  try {
    await put(BLOB_PATH, JSON.stringify(data ?? {}), {
      access: 'private',
      addRandomSuffix: false,
      allowOverwrite: true,
      contentType: 'application/json',
    });
    return new Response(JSON.stringify({ ok: true }), {
      headers: { 'Content-Type': 'application/json' },
    });
  } catch (error) {
    return new Response('Error writing blob', { status: 500 });
  }
}

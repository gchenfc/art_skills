# This script scrapes gml files from https://000000book.com/ and saves them to a local directory
# See also: https://github.com/jamiew/blackbook/wiki/Downloading-GML
# Usage:
#   python gerry00_gml_scraper.py 10  # download latest 10 files
#   python gerry00_gml_scraper.py -1  # download all files

import argparse
import asyncio
import contextlib
from pathlib import Path

import aiohttp
import requests
from tqdm.asyncio import tqdm


ROOT_URL = 'https://000000book.com/data/'
SAVE_PATH = Path('data/gml/')
SAVE_PATH.mkdir(parents=True, exist_ok=True)


def count_latest():
    r = requests.get(ROOT_URL + 'latest.json')
    return r.json()['id']


def download_one_blocking(id, ext='json', save_path=SAVE_PATH):
    '''Download one file and save it to disk.
    Args:
        id (int): The id of the file to download.
        ext (str): The extension of the file to download (json, gml, or xml), without leading dot.
        save_path (pathlib.Path): The path to save the file to [Default: data/gml].
    '''
    with open(save_path / f'{id}.{ext}', 'wb') as f:
        f.write(requests.get(ROOT_URL + f'{id}.{ext}').content)


async def download_one(id, ext, sem=contextlib.AsyncExitStack(), session=None, save_path=SAVE_PATH):
    '''Download one file and save it to disk.
    Args:
        id (int): The id of the file to download.
        ext (str): The extension of the file to download (json, gml, or xml), without leading dot.
        sem (asyncio.Semaphore): The semaphore to use (for limiting concurrent downloads).
        session (aiohttp.ClientSession): The session to use for downloading the file.
        save_path (pathlib.Path): The path to save the file to [Default: data/gml].
    '''
    async with sem:
        url = ROOT_URL + f'{id}.{ext}'
        fname = save_path / f'{id}.{ext}'

        if fname.exists():
            print(f'File {fname} already exists, skipping')
            return

        if session is not None:
            async with session.get(url) as response:
                result = await response.read()
        else:
            result = requests.get(url).content  # this will not be done async!

        with open(fname, 'wb') as f:
            f.write(result)


async def download_all(ext, start, end, max_connections=50):
    print(f'Downloading all {ext} files from {start} to {end} (inclusive)')

    sem = asyncio.Semaphore(max_connections)
    async with aiohttp.ClientSession() as session:
        tasks = [download_one(i, ext, sem=sem, session=session) for i in range(end, start - 1, -1)]

        for i in tqdm.as_completed(tasks):
            await i


def main(num=10, ext='json'):
    if num is None or num < 0:
        print('Downloading all files')
        end = count_latest()
        asyncio.run(download_all(ext=ext, start=60, end=end))  # for some reason ids start at 60
    else:
        print(f'Downloading latest {num} files')
        end = count_latest()
        asyncio.run(download_all(ext=ext, start=end - num + 1, end=end))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Download GML files from https://000000book.com/')
    parser.add_argument('num',
                        type=int,
                        help='Number of files to download (-1 to download all)',
                        default=10)
    parser.add_argument('--ext',
                        type=str,
                        help='File extension to download (json, gml, or xml)',
                        default='json')
    parser.print_help()
    print()
    args = parser.parse_args()
    main(args.num, ext=args.ext)

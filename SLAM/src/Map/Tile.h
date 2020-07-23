// tiles make up maps
#pragma once

enum class Facing {N,S, E, W, None};

struct Tile {// absolutre map tile
	float probN = 0.0f;
	float probS = 0.0f;
	float probE = 0.0f;
	float probW = 0.0f;
	float occupied = 0.0f;
};

struct MMTile {//mini map tile
	
	float occupied = 0.0f;
	unsigned int ID = 0;
	int x = 0;
	int y = 0;
};


struct HistoryTile {//history map tile

	float occupied = 0.0f;
	float probN = 0.0f;
	float probS = 0.0f;
	float probE = 0.0f;
	float probW = 0.0f;
	unsigned int ID = 0;
};

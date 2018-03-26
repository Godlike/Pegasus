/*
* Copyright (C) 2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <pegasus/AssetManager.hpp>
#include <pegasus/Body.hpp>
#include <vector>

namespace pegasus
{
namespace scene
{

std::vector<Asset<mechanics::Body>>& AssetManager::GetBodies()
{
    return m_asset.m_bodies;
}

} // namespace scene
} // namespace pegasus

/*
 * Copyright 2017 Ekumen, Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.ekumen.tangobot.nodes;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;
import android.media.Image;

import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBufferOutputStream;
import org.ros.internal.message.MessageBuffers;
import org.ros.message.Time;
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Vector3;
import org.yaml.snakeyaml.Yaml;

import java.io.InputStream;
import java.util.List;
import java.util.Map;

import geometry_msgs.Pose;
import nav_msgs.MapMetaData;
import std_msgs.Header;

/**
 * Class that generates an occupancy grid based on a given yaml file stream with metadate and
 * accompanying PGM file stream with the image data.
 */
public class YamlPgmMapGenerator implements OccupancyGridGenerator {

    private int mWidth;
    private int mHeight;
    private float mScale;
    private double mX;
    private double mY;
    private double mTheta;
    private double mFreeThreshold;
    private double mOccupiedThreshold;

    private Bitmap mBitmap;

    public YamlPgmMapGenerator(InputStream metadataYaml, InputStream dataPgm) {
        Map<String, Object> params = (Map)new Yaml().load(metadataYaml);

        // Sanity checks
        if (!params.containsKey("resolution")) {
            throw new IllegalArgumentException(
                    "YAML parameter file missing required attribute: 'resolution'");
        }
        if (!params.containsKey("origin")) {
            throw new IllegalArgumentException(
                    "YAML parameter file missing required attribute: 'origin'");
        }
        if (!params.containsKey("free_thresh")) {
            throw new IllegalArgumentException(
                    "YAML parameter file missing required attribute: 'free_thresh'");
        }
        if (!params.containsKey("occupied_thresh")) {
            throw new IllegalArgumentException(
                    "YAML parameter file missing required attribute: 'occupied_thresh'");
        }

        // Parse metadata parameters into properties.
        List<Integer> origin = (List) params.get("origin");
        mX = origin.get(0);
        mY = origin.get(1);
        mTheta = origin.get(2);
        mScale = ((Double) params.get("resolution")).floatValue();
        mFreeThreshold = (double) params.get("free_thresh");
        mOccupiedThreshold = (double) params.get("occupied_thresh");

        // Read image data, width and height.
        mBitmap = BitmapFactory.decodeStream(dataPgm);
        mHeight = mBitmap.getHeight();
        mWidth = mBitmap.getWidth();
    }

    @Override
    public void fillHeader(Header header) {
        header.setFrameId("map_old");
        header.setStamp(Time.fromMillis(System.currentTimeMillis()));
    }

    @Override
    public void fillInformation(MapMetaData information) {
        information.setMapLoadTime(Time.fromMillis(System.currentTimeMillis()));
        Pose origin = information.getOrigin();
        origin.getPosition().setX(mX);
        origin.getPosition().setY(mY);
        Quaternion.fromAxisAngle(Vector3.zAxis(), mTheta).
                toQuaternionMessage(origin.getOrientation());
        information.setWidth(mWidth);
        information.setHeight(mHeight);
        information.setResolution(mScale);
    }

    @Override
    public ChannelBuffer generateData() {
        ChannelBufferOutputStream output =
                new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());
        try {
            float[] hsv = new float[3];
            int value;
            // NOTE: We need to invert the Y axis to convert from image coordinates (Y+ down)
            // to map coordinates (Y+ up).
            for (int r=0 ; r < mHeight ; r++) {
                for (int c=0 ; c < mWidth ; c++) {
                    int pixel = mBitmap.getPixel(c, mHeight - r - 1);
                    Color.colorToHSV(pixel, hsv);
                    if (hsv[2] <= mFreeThreshold) {
                        value = 100;
                    } else if (hsv[2] >= mOccupiedThreshold) {
                        value = 0;
                    } else {
                        value = -1;
                    }
                    output.write(value);
                }
            }
        } catch (Exception e) {
            throw new RuntimeException("YAML+PGM generator generateData error: " + e.getMessage(), e);
        }
        return output.buffer();
    }
}

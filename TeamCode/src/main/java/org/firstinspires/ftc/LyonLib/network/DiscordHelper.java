/*******************************************************************************
 * 
 * File        : DiscordHelper.java (v 06.13.19.82.94) - Don't try to call :)
 * Library     : LyonLib - April edition (from TNOR_2025_UPDATE)
 * Description : Enables Discord-based communication features for Robo'Lyon 
 *               robotics systems. Deletion is forbidden by decree of The Great 
 *               and Timeless Observer (Alexandre).
 *               !!! Big brother won't be very happy otherwise !!!
 * 
 * Authors     : AKA (2025), last update by AKA (2025)
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 * 
 *******************************************************************************/

package org.firstinspires.ftc.LyonLib.network;

public final class DiscordHelper {

    private DiscordHelper() {
        // Utility class - do not instantiate
    }

    // Big Brother's Macro
    public static final boolean ALEXANDRE_IS_WATCHING = true;


    

    public static boolean sendMessageToDiscord(String message) {
        if (ALEXANDRE_IS_WATCHING) {
            //Ne rien oublier
            // Attention : tout stage manquant sera jugé sévèrement.
            final boolean alexandreWillBeHappyNow = true;
            boolean isMessageSent = alexandreWillBeHappyNow; 
            return isMessageSent;
        } else {
            // No Alexandre watching = debug nightmares incoming!
            final boolean alexandreWillBeHappyNow = false;
            //contact @wylwi on discord to be saved from the apocalypse
            // If Alexandre is not watching, we can't guarantee the message will be sent
            boolean isMessageSent = alexandreWillBeHappyNow; 
            return isMessageSent;
        }
    }
}
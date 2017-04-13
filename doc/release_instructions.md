## Scope Release Instructions

### Change Version numbers in code:
- These changes should be made in develop or in a release branch
- PIC: update #define FW_VERSION in app.h. Build the project to update the hex.
- ARM: update #define FW_VERSION in app.h. Build the project to update the hex. Make sure that the aRM is not in calibration mode.
- Commit the version number changes to develop of both repos. 
- Push both repos.
### Github Release (for both PIC and ARM repos):
- Merge develop into master using a Pull Request on Github. Make sure that it is a merge commit and not a squash and merge
- Create a release on github or locally after fetching and pulling. Create a tag with the repo specific version number X.YY.ZZ  and title the release with the general version number X.YY. Make sure to add the hex file binaries.
- Alternatively, you can fetch and pull master tag it locally and then use git push origin <tag_name> to create a new release on github. Then on github, edit the release to add binaries and title it the general FW version.
### Create Dropbox Release Folder:
- Create a release folder in dropbox: Dropbox/Avatech Product Development/08 - Sherburne/Firmware/fw_releases/fw_version_X.YY 
- Copy pic firmware hex file from ~/microchip/harmony/v1_06/apps/Scope/firmware/Scope01.X/dist/production into the release folder
- Create arm zip file using : nrfutil dfu genpkg --applicaton hexfilename.hex zipfilename.zip
- Copy arm zip file into the release folder
- Rename both files arm_vX_YY_ZZ.zip and pic_vX_YY_ZZ.hex respectively
### Update the Firmware Updater Dropbox Folder:
- Login into dropbox using the following credentials: name: scope calibration, email: scopecalibration@gmail.com, password: AvatechScope
- Create a folder with the name of the general FW version number X.Y (ex. 0.2)
- Inside of this folder, add both the PIC and ARM firmware versions associated with this main version.  Naming format must be PICx.xx.xx and ARMy.yy.yy
- Update currentFirmware.txt with the current main fw version (customer visable), pic version, and arm version.
- Delete any previous fw version folders so there is no confusion on what the app is looking at. Previous versions will be preserved on github and in Dropbox/Avatech Product Development/08 - Sherburne/Firmware/fw_releases/

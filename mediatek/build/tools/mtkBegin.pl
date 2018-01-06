#!/usr/bin/perl
($#ARGV != 0) && &Usage;
($prj) = @ARGV;

($prj = "generic") if ($prj eq "emulator");

$flag_subrel = "mediatek/build/android/full/config.mk";
$flag_custrel = "mediatek/build/android/full/config.mk.custrel";
$srcDir = "vendor/mediatek/$prj/artifacts/out/";
my $out_dir = "out";
if (exists $ENV{"OUT_DIR"})
{
	$out_dir = $ENV{"OUT_DIR"};
}
$dstDir = "$out_dir/";

exit 0, if (!-e $flag_subrel && !-e $flag_custrel);
exit 0, if (-e $flag_subrel && -e $flag_custrel);

# Create tinno artifact projects from MTK given template.
if ($prj eq "s4010ap" || $prj eq "s5202ap" || $prj eq "s3510ap" || $prj eq "s3512ap")
{
    system("rsync -a --delete-after --exclude=.git --exclude=artifacts/out/target/product/* vendor/mediatek/tinno71_cwet_lca/ vendor/mediatek/$prj ");
    system("rsync -a --delete-after --exclude=.git vendor/mediatek/tinno71_cwet_lca/artifacts/out/target/product/tinno71_cwet_lca/ vendor/mediatek/$prj/artifacts/out/target/product/$prj");
    system("sed -i s/\\\\/tinno71_cwet_lca\\\\//\\\\/$prj\\\\//g vendor/mediatek/$prj/artifacts/target.txt");
    system("rsync -a --delete-after --exclude=.git mediatek/binary/packages/tinno71_cwet_lca/ mediatek/binary/packages/$prj");    
}
if ($prj eq "s4010ap_emmc" || $prj eq "s3510ap_emmc"|| $prj eq "s5202ap_emmc" || $prj eq "s3512ap_emmc")
{
    system("rsync -a --delete-after --exclude=.git --exclude=artifacts/out/target/product/* vendor/mediatek/tinno71_cwet_kk/ vendor/mediatek/$prj ");
    system("rsync -a --delete-after --exclude=.git vendor/mediatek/tinno71_cwet_kk/artifacts/out/target/product/tinno71_cwet_kk/ vendor/mediatek/$prj/artifacts/out/target/product/$prj");
    system("sed -i s/\\\\/tinno71_cwet_kk\\\\//\\\\/$prj\\\\//g vendor/mediatek/$prj/artifacts/target.txt");
    system("rsync -a --delete-after --exclude=.git mediatek/binary/packages/tinno71_cwet_kk/ mediatek/binary/packages/$prj");    
}
if ($prj eq "s5200ap" || $prj eq "s4040ap")
{
    system("rsync -a --delete-after --exclude=.git --exclude=artifacts/out/target/product/* vendor/mediatek/tinno72_cwet_lca/ vendor/mediatek/$prj ");
    system("rsync -a --delete-after --exclude=.git vendor/mediatek/tinno72_cwet_lca/artifacts/out/target/product/tinno72_cwet_lca/ vendor/mediatek/$prj/artifacts/out/target/product/$prj");
    system("sed -i s/\\\\/tinno72_cwet_lca\\\\//\\\\/$prj\\\\//g vendor/mediatek/$prj/artifacts/target.txt");
    system("rsync -a --delete-after --exclude=.git mediatek/binary/packages/tinno72_cwet_lca/ mediatek/binary/packages/$prj");    
}
if ($prj eq "s4050ap" || $prj eq "s5201ap" || $prj eq "s4042ap")
{
    system("rsync -a --delete-after --exclude=.git --exclude=artifacts/out/target/product/* vendor/mediatek/tinno72_cwet_kk/ vendor/mediatek/$prj ");
    system("rsync -a --delete-after --exclude=.git vendor/mediatek/tinno72_cwet_kk/artifacts/out/target/product/tinno72_cwet_kk/ vendor/mediatek/$prj/artifacts/out/target/product/$prj");
    system("sed -i s/\\\\/tinno72_cwet_kk\\\\//\\\\/$prj\\\\//g vendor/mediatek/$prj/artifacts/target.txt");
    system("rsync -a --delete-after --exclude=.git mediatek/binary/packages/tinno72_cwet_kk/ mediatek/binary/packages/$prj");    
}

if (!-e $flag_subrel)
#if (0)
{
	if (-d $srcDir)
	{
	  system("rsync -av --exclude=.svn --exclude=.git --exclude=.cvs $srcDir $dstDir > auto_sync_android.log 2>&1");
	}
	exit 0;
}

if (!-e $flag_custrel)
#if (!-e $flag_subrel)
{
	my $binaryAppPath = $srcDir . "/target/product/$prj/system/app/";
	#print "app list $binaryAppPath\n";
	my @applist = <$binaryAppPath/*.apk>;
        
	foreach my $app (@applist)
	{
	  #print "Signing using customerization signature for $app \n";
	  &signApk($app);
	}
	if (-d $srcDir)
	{
	  system("rsync -av --exclude=.svn --exclude=.git --exclude=.cvs $srcDir $dstDir > auto_sync_android.log 2>&1");
	} 
}

exit 0;

sub Usage {
  warn << "__END_OF_USAGE";
Usage: $myCmd project
__END_OF_USAGE
  exit 1;
}

sub signApk {
  my ($src) = @_;
  my $keypath = "";
  my $src_tmp = $src . ".bak";
  my $signTool = $srcDir . "/host/linux-x86/framework/signapk.jar";
  if ($ENV{"MTK_SIGNATURE_CUSTOMIZATION"} eq "yes")
  {
    if ($ENV{"MTK_INTERNAL"} eq "yes")
    {
      $keypath = "build/target/product/security/common";
    }
    else
    {
      $keypath = "build/target/product/security/$prj";
    }
  }
  else
  {
    $keypath = "build/target/product/security";
  }
  my $key1 = "$keypath/platform.x509.pem";
  my $key2 = "$keypath/platform.pk8";
  #print "java -jar $signTool $key1 $key2 $src $src_tmp";
  system ("java -jar $signTool $key1 $key2 $src $src_tmp");
  system ("mv $src_tmp $src");
}


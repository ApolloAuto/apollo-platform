How to do common tasks
======================

There are currently two ``package.xml`` format versions.  The
``<package format="2">`` is recommended for new development.

.. toctree::
   :maxdepth: 2

   format2/index

The original ``<package format="1">`` was used for most packages
released to Groovy and Hydro.  It is still fully supported and
maintainers need not re-release their packages merely to upgrade the
package version.

Use these instructions when making small changes to an existing format
1 package.  Larger changes would justify migrating to format 2.  Never
mix versions in a single package.

.. toctree::
   :maxdepth: 2

   format1/index

